# Информация о разработке

## Установка ROS 2 Jazzy

```
locale  

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```
sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

```
sudo apt update -y && sudo apt install ros-dev-tools -y
```

```
sudo apt update -y && sudo apt upgrade -y
```

```
sudo apt install ros-jazzy-desktop -y
```

```
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

```
sync
reboot
```

## Установка симулятора

### Скачивание симулятора

Для того, чтобы скачать симулятор откройте терминал и выполните следующие команды:

```
cd
git clone --recurse-submodules https://github.com/voltbro/roverchallenge-ru.git
echo "source ~/roverchallenge-ru/source setup.sh" >> ~/.bashrc
```

### Установка зависимостей

Для начала необходимо установить пакетные (APT) зависимости. Для этого откройте терминал и выполните следующую команду:

```
sudo apt update && sudo apt upgrade && sudo apt install python3-pip jsonnet just ros-jazzy-joy ros-jazzy-ros2-control ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-moveit-planners-chomp ros-jazzy-imu-tools can-utils -y
```

```
echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> ~/.bashrc
reboot
```

Далее необходимо установить ROS зависимости. Для этого в терминале выполните следующие команды:

```
sudo rosdep init
rosdep update
```

Перезапустите терминал и выполните следующую команду:

```
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

## запуск

Для запуска симуляции в этом же терминале:

```
just sim world:=worlds/field/field.sdf
```
