# Информация о разработке

## Зависимости

**APT**: `sudo apt update && sudo apt upgrade && sudo apt install just ros-jazzy-joy ros-jazzy-ros2-control ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-moveit-planners-chomp ros-jazzy-imu-tools can-utils`

**rosdep**: `rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y`

## Первый запуск

- `source setup.sh`
- `just build`
- Пересоздать терминал
- `source setup.sh`
- `just sim world:=worlds/field/field.sdf`

## Обычный запуск

Один раз в начале работы:
- `source setup.sh`

Для запуска:
- `just sim world:=worlds/field/field.sdf`
