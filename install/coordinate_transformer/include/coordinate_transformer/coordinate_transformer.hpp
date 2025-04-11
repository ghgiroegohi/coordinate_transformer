#ifndef COORDINATE_TRANSFORMER_HPP
#define COORDINATE_TRANSFORMER_HPP

#include <string>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace coordinate_transformer {

/**
 * @brief Перечисление статусов результата операций.
 */
enum class ResultStatus {
  SUCCESS,
  OUT_OF_BOUNDS,
  TRANSFORM_NOT_FOUND,
  INVALID_INPUT
};

/**
 * @brief Класс для преобразования координат между системами отсчёта.
 *
 * Данный класс предоставляет методы для прямых и обратных преобразований,
 * загрузки трансформаций из YAML-конфигурации, ручного добавления трансформаций,
 * установки границ и проверки выхода за пределы установленных границ.
 */
class CoordinateTransformer {
public:
  /**
   * @brief Конструктор.
   * @param node Указатель на rclcpp::Node для доступа к параметрам и логированию.
   */
  explicit CoordinateTransformer(const rclcpp::Node::SharedPtr & node);

  /**
   * @brief Прямое преобразование из исходной системы в целевую.
   * 
   * @param input Входная поза с указанием исходного фрейма.
   * @param output Выходная поза в целевом фрейме.
   * @param target_frame Целевая система отсчёта.
   * @return ResultStatus::SUCCESS в случае успеха или соответствующий статус ошибки.
   */
  ResultStatus convert(const geometry_msgs::msg::PoseStamped & input,
                       geometry_msgs::msg::PoseStamped & output,
                       const std::string & target_frame);

  /**
   * @brief Обратное преобразование из целевой системы в исходную.
   * 
   * @param input Входная поза в целевом фрейме.
   * @param output Выходная поза в исходном фрейме.
   * @param source_frame Исходная система отсчёта.
   * @return ResultStatus::SUCCESS в случае успеха или соответствующий статус ошибки.
   */
  ResultStatus inverseConvert(const geometry_msgs::msg::PoseStamped & input,
                              geometry_msgs::msg::PoseStamped & output,
                              const std::string & source_frame);

  /**
   * @brief Загрузка трансформаций из YAML-файла.
   *
   * Ожидается, что YAML-файл содержит раздел "transforms" с параметрами:
   * parent_frame, child_frame, translation (x, y, z) и rotation (x, y, z, w).
   * 
   * @param config_file Путь к YAML-файлу.
   */
  void loadConfig(const std::string & config_file);

  /**
   * @brief Ручное добавление трансформации.
   * @param transform Трансформация в виде geometry_msgs::msg::TransformStamped.
   */
  void addTransform(const geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief Установка границ для заданной системы отсчёта.
   * @param frame_id Идентификатор фрейма.
   * @param min Минимальные координаты.
   * @param max Максимальные координаты.
   */
  void setBounds(const std::string & frame_id,
                 const geometry_msgs::msg::Point & min,
                 const geometry_msgs::msg::Point & max);

private:
  rclcpp::Node::SharedPtr node_;
  // Словарь трансформаций: ключ – "parent->child", значение – tf2::Transform
  std::unordered_map<std::string, tf2::Transform> transforms_;
  
  // Структура для хранения границ
  struct Bounds {
    geometry_msgs::msg::Point min;
    geometry_msgs::msg::Point max;
  };
  // Словарь границ для каждого frame_id
  std::unordered_map<std::string, Bounds> bounds_;

  /**
   * @brief Проверка, что позиция находится в пределах установленных границ.
   * @param pose Поза для проверки.
   * @param frame_id Фрейм, для которого проверяются границы.
   * @return true, если координаты в пределах, иначе false.
   */
  bool checkBounds(const geometry_msgs::msg::PoseStamped & pose, const std::string & frame_id);
};

} 

#endif // COORDINATE_TRANSFORMER_HPP