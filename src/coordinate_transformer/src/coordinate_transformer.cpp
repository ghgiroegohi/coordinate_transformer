#include "coordinate_transformer/coordinate_transformer.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include "tf2/LinearMath/Quaternion.h"

namespace coordinate_transformer {

CoordinateTransformer::CoordinateTransformer(const rclcpp::Node::SharedPtr & node)
  : node_(node) {
  if (!node_) {
    throw std::runtime_error("Node pointer is null");
  }
}

ResultStatus CoordinateTransformer::convert(const geometry_msgs::msg::PoseStamped & input,
                                              geometry_msgs::msg::PoseStamped & output,
                                              const std::string & target_frame) {
  // Формируем ключ для поиска трансформации
  std::string key = input.header.frame_id + "->" + target_frame;
  if (transforms_.find(key) == transforms_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Transform %s not found", key.c_str());
    return ResultStatus::TRANSFORM_NOT_FOUND;
  }
  tf2::Transform transform = transforms_[key];

  tf2::Transform input_tf;
  tf2::fromMsg(input.pose, input_tf);

  tf2::Transform result_tf = transform * input_tf;

  output.header.frame_id = target_frame;
  tf2::toMsg(result_tf, output.pose);

  if (!checkBounds(output, target_frame)) {
    return ResultStatus::OUT_OF_BOUNDS;
  }
  return ResultStatus::SUCCESS;
}

ResultStatus CoordinateTransformer::inverseConvert(const geometry_msgs::msg::PoseStamped & input,
                                                     geometry_msgs::msg::PoseStamped & output,
                                                     const std::string & source_frame) {
  // Формируем ключ для обратного поиска трансформации
  std::string key = source_frame + "->" + input.header.frame_id;
  if (transforms_.find(key) == transforms_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "Transform %s not found", key.c_str());
    return ResultStatus::TRANSFORM_NOT_FOUND;
  }
  tf2::Transform transform = transforms_[key];

  // Инвертируем трансформацию
  tf2::Transform inv_transform = transform.inverse();

  tf2::Transform input_tf;
  tf2::fromMsg(input.pose, input_tf);

  tf2::Transform result_tf = inv_transform * input_tf;

  output.header.frame_id = source_frame;
  geometry_msgs::msg::Transform transform_msg;
  tf2::toMsg(result_tf, transform_msg);
  output.pose.position.x = transform_msg.translation.x;
  output.pose.position.y = transform_msg.translation.y;
  output.pose.position.z = transform_msg.translation.z;
  output.pose.orientation.x = transform_msg.rotation.x;
  output.pose.orientation.y = transform_msg.rotation.y;
  output.pose.orientation.z = transform_msg.rotation.z;
  output.pose.orientation.w = transform_msg.rotation.w;

  if (!checkBounds(output, source_frame)) {
    return ResultStatus::OUT_OF_BOUNDS;
  }
  return ResultStatus::SUCCESS;
}

void CoordinateTransformer::loadConfig(const std::string & config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  // Ожидаем, что конфигурация содержит массив трансформаций
  for (const auto & trans_node : config["transforms"]) {
    std::string parent_frame = trans_node["parent_frame"].as<std::string>();
    std::string child_frame = trans_node["child_frame"].as<std::string>();
    auto translation = trans_node["translation"];
    auto rotation = trans_node["rotation"];

    tf2::Transform transform;
    tf2::Vector3 origin(translation["x"].as<double>(),
                        translation["y"].as<double>(),
                        translation["z"].as<double>());
    tf2::Quaternion quat(rotation["x"].as<double>(),
                         rotation["y"].as<double>(),
                         rotation["z"].as<double>(),
                         rotation["w"].as<double>());
    transform.setOrigin(origin);
    transform.setRotation(quat);

    std::string key = parent_frame + "->" + child_frame;
    transforms_[key] = transform;
  }
}

void CoordinateTransformer::addTransform(const geometry_msgs::msg::TransformStamped & transform_msg) {
  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);
  std::string key = transform_msg.header.frame_id + "->" + transform_msg.child_frame_id;
  transforms_[key] = transform;
}

void CoordinateTransformer::setBounds(const std::string & frame_id,
                                        const geometry_msgs::msg::Point & min,
                                        const geometry_msgs::msg::Point & max) {
  bounds_[frame_id] = {min, max};
}

bool CoordinateTransformer::checkBounds(const geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id) {
  if (bounds_.find(frame_id) == bounds_.end()) {
    // Если границы не заданы, считаем, что всё в пределах
    return true;
  }
  const Bounds & b = bounds_[frame_id];
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double z = pose.pose.position.z;
  if (x < b.min.x || x > b.max.x ||
      y < b.min.y || y > b.max.y ||
      z < b.min.z || z > b.max.z) {
    return false;
  }
  return true;
}

} 