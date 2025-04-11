Пример использование:

cd ~/ros2_ws

colcon build --packages-select coordinate_transformer

source install/setup.bash

colcon test --packages-select coordinate_transformer

colcon test-result --verbose
