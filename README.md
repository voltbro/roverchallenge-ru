# Симулятор космического ровера "Исследователь" с манипулятором RUKA на ROS2

Данный симулятор предназначен для виртуальной работы с реальными роботами проекта "Братья Вольт":

- Роботизированная платорма "Исследователь": https://voltbro.ru/explorer
- Образовательный манипулятор RUKA: https://shop.voltbro.ru/robots#!/tproduct/494042486-1715606902155

## Установка ROS 2 Jazzy

Перед установкой, проверьте наличие обновлений уже установленных пакетов и установите их, если они есть:

```
sudo apt update -y && sudo apt upgrade -y
```
###  Установка локали

Убедитесь, что у вас установлена локаль, поддерживающая UTF-8. Если вы находитесь в минимальной среде (например, в контейнере Docker), локаль может быть минимальной, например **POSIX**:

```
locale  

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

###  Включение необходимых репозиториев

Добавьте репозиторий ROS 2 apt в вашу систему.

Сначала убедитесь, что репозиторий Ubuntu Universe включен:

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Теперь добавьте GPG-ключ ROS 2 с помощью apt:

```
sudo apt update -y && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Затем добавьте репозиторий в список источников:

```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Установка инструментов разработки

Если вы собираетесь создавать пакеты ROS или заниматься разработкой иным образом, вы также можете установить инструменты разработки:

```
sudo apt update -y && sudo apt install ros-dev-tools -y
```

### Установка ROS 2

Обновите кэши репозиториев apt после настройки репозиториев. Пакеты ROS 2 создаются на основе часто обновляемых систем Ubuntu. Перед установкой новых пакетов всегда рекомендуем убедиться, что ваша система обновлена:

```
sudo apt update -y && sudo apt upgrade -y
```

Установка полной версии ROS 2 Jazzy:

```
sudo apt install ros-jazzy-desktop -y
```

### Настройка рабочего окружения

Настройка рабочих параметров ROS, происходит через установку переменных окружения (например пути библиотек, адреса серверов и тд). Эту операцию можно делать руками, но проще настроить их автоматический экспорт при запуске интерактивной оболочки bash.

Добавьте переменные окружения ROS, для их автоматической установки при запуске bash:

```
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

Синхронизируйте установленные файлы и перезагрузите компьютер:

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
echo "source ~/roverchallenge-ru/setup.sh" >> ~/.bashrc
```

### Установка зависимостей

Для начала необходимо установить пакетные (APT) зависимости. Для этого откройте терминал и выполните следующую команду:

```
sudo apt update && sudo apt upgrade && sudo apt install python3-pip jsonnet just ros-jazzy-joy ros-jazzy-ros2-control ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros-gz ros-jazzy-moveit ros-jazzy-moveit-planners-chomp ros-jazzy-imu-tools can-utils -y
```

```
echo "export PIP_BREAK_SYSTEM_PACKAGES=1" >> ~/.bashrc
```

Синхронизируйте установленные файлы и перезагрузите компьютер:

```
sync
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

Синхронизируйте установленные файлы и перезагрузите компьютер:

```
sync
reboot
```

## Первый запуск симуляции

Для запуска симуляции перейдите в папку с симулятором и запустите его:

```
cd ~/roverchallenge-ru/
just sim
```

**Обратите внимание, что при первом запуске в терминале будет ошибка запуска. Не пугайтесь, перезапустите терминал и выполните команды к запуску еще раз! Мы знаем об этой проблеме и полны решимости её победить**

## Запуск симуляции

Для запуска симуляции перейдите в папку с симулятором и запустите его:

```
cd ~/roverchallenge-ru/
just sim
```

После запуска вы должны увидеть симулятор Gazebo и среду визуализации RViz:

![Simulator_ROS2](https://github.com/user-attachments/assets/9a479dd4-403d-4034-8b4d-8b78eca259ab)
