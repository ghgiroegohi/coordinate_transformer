/**
 * @file test_coordinate_transformer.cpp
 * @brief Тесты для CoordinateTransformer
 */

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "coordinate_transformer/coordinate_transformer.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace coordinate_transformer;

  /**
 * @class CoordinateTransformerTest
 * @brief Класс тестов для CoordinateTransformer
 */

class CoordinateTransformerTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<CoordinateTransformer> transformer;

  /**
   * @brief Установка начальных условий для тестов
   */

  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node = rclcpp::Node::make_shared("test_node");
    transformer = std::make_shared<CoordinateTransformer>(node);

    // Добавление трансформации: frameA -> frameB, смещение на (1,0,0)
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "frameA";
    tf_msg.child_frame_id = "frameB";
    tf_msg.transform.translation.x = 1.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    transformer->addTransform(tf_msg);

    // Добавление трансформации: frameA -> frameA
    geometry_msgs::msg::TransformStamped tf_msg_same_frame;
    tf_msg_same_frame.header.frame_id = "frameA";
    tf_msg_same_frame.child_frame_id = "frameA";
    tf_msg_same_frame.transform.translation.x = 0.0;
    tf_msg_same_frame.transform.translation.y = 0.0;
    tf_msg_same_frame.transform.translation.z = 0.0;
    tf_msg_same_frame.transform.rotation.x = 0.0;
    tf_msg_same_frame.transform.rotation.y = 0.0;
    tf_msg_same_frame.transform.rotation.z = 0.0;
    tf_msg_same_frame.transform.rotation.w = 1.0;
    transformer->addTransform(tf_msg_same_frame);

    // Установка границ для frameB
    geometry_msgs::msg::Point min;
    min.x = 0.0; min.y = -1.0; min.z = -1.0;
    geometry_msgs::msg::Point max;
    max.x = 5.0; max.y = 1.0; max.z = 1.0;
    transformer->setBounds("frameB", min, max);
  }

  /**
   * @brief Уничтожение тестов
   */

  void TearDown() override {
    rclcpp::shutdown();
  }
};

  /**
 * @test Тест функции преобразования координат
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, TransformTest) {
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));
  transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  geometry_msgs::msg::Transform transform_msg;
  tf2::toMsg(transform, transform_msg);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "frameA";
  tf_msg.child_frame_id = "frameB";
  tf_msg.transform = transform_msg;

  transformer->addTransform(tf_msg);

  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 0.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = 0.0;
  input.pose.orientation.y = 0.0;
  input.pose.orientation.z = 0.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::OUT_OF_BOUNDS);
  EXPECT_NEAR(output.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(output.pose.position.y, 2.0, 1e-6);
  EXPECT_NEAR(output.pose.position.z, 3.0, 1e-6);
}

  /**
 * @test Тест функции преобразования координат с успешным преобразованием
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус успешного преобразования
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, ConvertSuccess) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 0.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = 0.0;
  input.pose.orientation.y = 0.0;
  input.pose.orientation.z = 0.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::SUCCESS);
  EXPECT_NEAR(output.pose.position.x, 1.0, 1e-6);
}

/**
 * @test Тест функции преобразования координат с выходом за пределы границ
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус выхода за пределы границ
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, OutOfBounds) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  // Координата, которая после преобразования выйдет за границы
  input.pose.position.x = 10.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::OUT_OF_BOUNDS);
}

/**
 * @test Тест функции преобразования координат с неизвестной трансформацией
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус неизвестной трансформации
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, TransformNotFound) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "nonexistent_frame";
  input.pose.position.x = 0.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::TRANSFORM_NOT_FOUND);
}

/**
 * @test Тест функции преобразования координат с одинаковой системой отсчёта
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус успешного преобразования
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, SameFrame) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 3.0;
  input.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameA");

  EXPECT_EQ(status, ResultStatus::SUCCESS);
  EXPECT_EQ(output.header.frame_id, "frameA");
  EXPECT_DOUBLE_EQ(output.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(output.pose.position.z, 3.0);
}

/**
 * @test Тест функции преобразования координат с неизвестной исходной системой отсчёта
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус неизвестной исходной системы отсчёта
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, SourceFrameNotFound) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "nonexistent_frame";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 3.0;
  input.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::TRANSFORM_NOT_FOUND);
}

/**
 * @test Тест функции преобразования координат с неизвестной целевой системой отсчёта
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус неизвестной целевой системы отсчёта
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, TargetFrameNotFound) {
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 1.0;
  input.pose.position.y = 2.0;
  input.pose.position.z = 3.0;
  input.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "nonexistent_frame");

  EXPECT_EQ(status, ResultStatus::TRANSFORM_NOT_FOUND);
}

/**
 * @test Тест функции преобразования координат с поворотом на 90 градусов
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус успешного преобразования
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, Rotation90) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "frameA";
  tf_msg.child_frame_id = "frameB";
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.707;
  tf_msg.transform.rotation.w = 0.707;
  transformer->addTransform(tf_msg);

  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 1.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = 0.0;
  input.pose.orientation.y = 0.0;
  input.pose.orientation.z = 0.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::SUCCESS);
  EXPECT_NEAR(output.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(output.pose.position.y, 1.0, 1e-6);
}

/**
 * @test Тест функции преобразования координат с поворотом на 180 градусов
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус успешного преобразования
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, Rotation180) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "frameA";
  tf_msg.child_frame_id = "frameB";
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 1.0;
  tf_msg.transform.rotation.w = 0.0;
  transformer->addTransform(tf_msg);

  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 1.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = 0.0;
  input.pose.orientation.y = 0.0;
  input.pose.orientation.z = 0.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::OUT_OF_BOUNDS);
  EXPECT_NEAR(output.pose.position.x, -1.0, 1e-6);
  EXPECT_NEAR(output.pose.position.y, 0.0, 1e-6);
}

/**
 * @test Тест функции преобразования координат с поворотом на 270 градусов
 *
 * Этот тест проверяет, что функция преобразования координат работает корректно
 * и возвращает статус успешного преобразования
 *
 * @param input Входная поза
 * @param output Выходная поза
 * @param target_frame Целевая система отсчёта
 * @return Статус преобразования
 */

TEST_F(CoordinateTransformerTest, Rotation270) {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.frame_id = "frameA";
  tf_msg.child_frame_id = "frameB";
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = -0.707;
  tf_msg.transform.rotation.w = 0.707;
  transformer->addTransform(tf_msg);

  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = "frameA";
  input.pose.position.x = 1.0;
  input.pose.position.y = 0.0;
  input.pose.position.z = 0.0;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = 0.0;
  input.pose.orientation.y = 0.0;
  input.pose.orientation.z = 0.0;

  geometry_msgs::msg::PoseStamped output;
  ResultStatus status = transformer->convert(input, output, "frameB");

  EXPECT_EQ(status, ResultStatus::SUCCESS);
  EXPECT_NEAR(output.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(output.pose.position.y, -1.0, 1e-6);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
