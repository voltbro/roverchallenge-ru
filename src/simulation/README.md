# Симуляция для космического робота

## Глобальная настройка окружения

Работает на линукс хосте, vm или нативно.

Основные инструменты: [ROS2](https://docs.ros.org/en/jazzy/Installation.html), [GazeboSim](https://gazebosim.org/docs/all/getstarted).

> Версия ROS2 - **jazzy**, gazebo - **harmonic**.

## Инструменты для разработки

- **jsonnet**: шаблонизатор
- **just**: запускалка команд, типо vscode tasks но для командной строки. Ставим [отсюда](https://github.com/casey/just#packages) через prebuilt-mcr (предпоследняя строка)
- **xacro**: стандартная ros-тулза для работы с xml (ros-jazzy-xacro)
- **pip**: само собой. `sudo apt install python3-pip`

И добавьте `export PATH=$PATH:/home/igor/.local/bin` в `.bashrc`.
