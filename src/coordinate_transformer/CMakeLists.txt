cmake_minimum_required(VERSION 3.5)
project(coordinate_transformer)
set(CATKIN_INSTALL_INTO_PREFIX_ROOT OFF)

# Установка стандарта C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Поиск необходимых пакетов
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Объявление библиотеки
add_library(${PROJECT_NAME} SHARED
  src/coordinate_transformer.cpp
)

# Добавление директорий заголовочных файлов для цели
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${tf2_geometry_msgs_INCLUDE_DIRS}  # Добавьте эту строку
)

# Указание зависимостей для цели
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

# Добавление unit-тестов
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_coordinate_transformer.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    yaml-cpp
  )
endif()

# Установка библиотеки
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Установка заголовочных файлов
install(
  DIRECTORY include/
  DESTINATION include
)

# Экспорт информации о цели
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Завершение пакета
ament_package()
