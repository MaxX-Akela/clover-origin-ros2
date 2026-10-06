# Forked from [CopterExpress/clover](https://github.com/CopterExpress/clover)

> Это порт Clover с ROS 1 (Noetic) на **ROS 2 Jazzy**. Стабильную версию на ROS 1 ищите в [оригинальном репозитории](https://github.com/CopterExpress/clover).

# clover🍀 for ROS 2: create autonomous drones easily

<img src="docs/assets/clover42-main-margin.png" align="right" width="400px" alt="COEX Clover Drone">

Clover is an open source [ROS](https://www.ros.org)-based framework, providing user-friendly tools to control [PX4](https://px4.io)-powered drones. Clover is available as a ROS package, but is shipped mainly as a preconfigured image for Raspberry Pi. Once you've installed Raspberry Pi on your drone and flashed the image to its microSD card, taking the drone up in the air is a matter of minutes.

COEX Clover Drone is an educational programmable drone kit, suited perfectly for running clover software. The kit is shipped unassembled and includes Pixracer-compatible autopilot running PX4 firmware, Raspberry Pi 4 as a companion computer, a camera for computer vision navigation as well as additional sensors and peripheral devices. Batteries included.

The main documentation is available at [https://clovercoex.tech](https://clovercoex.tech/).


## Образ для Raspberry Pi

Преднастроенный образ с установленным и настроенным ПО, готовый к полёту, будет доступен [в разделе Releases](https://github.com/MaxX-Akela/clover-origin-ros2/releases).

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/MaxX-Akela/clover-origin-ros2/build-image.yaml?branch=master)
![GitHub all releases](https://img.shields.io/github/downloads/MaxX-Akela/clover-origin-ros2/total)

Состав образа:

* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
* Ubuntu 24.04
* OpenCV
* [`mavros`](https://github.com/mavlink/mavros) для ROS 2
* Пакет `aruco_pose` для навигации по маркерам
* Пакет `clover` для автономного управления дроном

> Целевая платформа образа — Raspberry Pi 4/5


## Поддержка

> По вопросам ROS 2 порта используйте [Issues](https://github.com/MaxX-Akela/clover-origin-ros2/issues) этого репозитория.
