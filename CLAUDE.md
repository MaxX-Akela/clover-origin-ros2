# CLAUDE.md — порт пакета clover на ROS 2 Jazzy

## Цель

Портировать пакет `clover` из ROS 1 (Noetic, catkin) на **ROS 2 Jazzy** (ament_cmake, colcon).
Оригинал: https://github.com/CopterExpress/clover (подкаталог `clover`), лицензия MIT,
Copyright (c) 2018 Copter Express Technologies.

Пакет `aruco_pose` в этом репозитории уже портирован и собирается. Его не трогать.
Пакеты `clover_blocks`, `clover_description`, `clover_simulation` в область не входят.

## Окружение

- WSL, Ubuntu 24.04, ROS 2 Jazzy (`source /opt/ros/jazzy/setup.bash`), репозиторий внутри `~/`.
- Сборка: `colcon build --packages-select clover --symlink-install`
- Тесты: `colcon test --packages-select clover && colcon test-result --verbose`
- FCU-слой: **mavros для ROS 2** (`ros-jazzy-mavros`, `ros-jazzy-mavros-extras`), а не uXRCE-DDS/px4_msgs.
  Все обращения к автопилоту держи в одном месте (в `simple_offboard.cpp` и нескольких узлах),
  не размазывай их по коду: позже слой могут заменить.
- `sudo` не используй. Если нужен apt-пакет, назови его и остановись.
- Железа и симуляции нет. Ничего не утверждай про полёт, говори только о том, что проверено сборкой и тестами.

## Что в области (всё внутри `clover/`)

| Файл | Объём | Заметка |
|---|---|---|
| `src/simple_offboard.cpp` | 1200 строк | ядро: сервисы navigate/set_position/... , самый сложный файл |
| `src/selfcheck.py` | 989 строк | rospy → rclpy |
| `src/led.cpp` | 335 | зависит от `led_msgs` |
| `src/optical_flow.cpp` | 308 | nodelet, dynamic_reconfigure (`cfg/Flow.cfg`) |
| `src/rc.cpp` | 186 | |
| `src/vpe_publisher.cpp` | 185 | позиция из aruco_map в FCU |
| `src/camera_markers.cpp` | 101 | |
| `src/shell.cpp` | 50 | сервис `Execute` |
| `src/clover/__init__.py` | 35 | `long_callback` на rospy |
| `msg/State.msg`, `srv/*.srv` (12 файлов) | | интерфейсы публичного API |
| `launch/*.launch`, `launch/mavros_config.yaml` | | в Python launch и параметры ROS 2 |
| `examples/*.py` | | пользовательский API, порт в конце |
| `test/` | | rostest → launch_testing |
| `src/autotest/`, `src/camera_stream`, `src/waitfile`, `src/www`, `udev/`, `camera_info/` | | скрипты и данные, править только нужное |

`launch/simulator.launch` не портировать. `www/` (веб-часть) делается отдельным этапом позже.

## Внешние зависимости, которых нет в Jazzy

- **`led_msgs`** (SetLEDs, LEDState, LEDStateArray): нужен для `led.cpp`. Создай интерфейсы с теми же
  именами и полями. Прежде чем что-то копировать из оригинального `led_msgs`, проверь его лицензию, при
  сомнении пиши своё определение.
- **Драйвер ленты** (`ws281x`) и **`vl53l1x`**: драйверов ROS 2 нет. Это отдельные этапы,
  на Raspberry Pi 5 `rpi_ws281x` и `pigpio` не работают (чип RP1). Здесь только оставь точки подключения
  (топики/сервисы `led/set_leds`, `led/state`, `rangefinder/range`) и скажи, что драйвера нет.
- **`cv_camera`**: в Jazzy нет. Для launch используй `v4l2_camera` (USB) как значение по умолчанию.
- **`tf2_web_republisher`**, **`web_video_server`**, **`rosbridge_server`**: проверь наличие в Jazzy
  (`apt-cache policy ros-jazzy-...`) и опиши в README, чего не хватает.
- `geographiclib`: нужен для `navigate_global`, ставь системный `libgeographiclib-dev`.

## Соответствия ROS 1 → ROS 2 (дополнительно к уже сделанному в aruco_pose)

| ROS 1 | ROS 2 Jazzy |
|---|---|
| `ros::NodeHandle`, `advertiseService` | `rclcpp::Node`, `create_service` |
| `ros::ServiceClient`, `serviceClient` | `create_client`, вызовы через `async_send_request`, без блокировки в колбэке |
| `ros::Timer`, `ros::Rate` | `create_wall_timer` или `create_timer` |
| `ros::topic::waitForMessage` | не блокируй колбэк: подписка + флаг готовности, или ожидание вне колбэков |
| `nh.param`, `getParam` | `declare_parameter`; `/` в именах заменяй на `.`; динамическое изменение через `add_on_set_parameters_callback` |
| `dynamic_reconfigure` (`Flow.cfg`: `enabled`) | параметр `enabled` |
| `rospy` | `rclpy`, один `MultiThreadedExecutor` там, где есть блокирующие вызовы |
| `rospy.ServiceProxy` | клиент `rclpy`, не вызывай `.call()` внутри колбэка |
| `nodelet` (`optical_flow`) | composable node (`rclcpp_components`) |
| `setup.py` + `catkin_python_setup` | `ament_cmake_python`: `ament_python_install_package(clover PACKAGE_DIR src/clover)` |
| `$(find pkg)`, `<arg>`, `<remap>` | `FindPackageShare`, `LaunchConfiguration`, `remappings` |

## Особенности mavros для ROS 2 (проверь по установленному пакету, не по памяти)

- Имена сервисов и топиков, типы в `mavros_msgs`, имена плагинов и формат параметров (`ros__parameters`)
  сверяй с установленным mavros: `ros2 interface show mavros_msgs/srv/...`, файлы в
  `/opt/ros/jazzy/share/mavros*`. Если чего-то нет или имя другое, запиши в `NOTES.md`.
- QoS у топиков mavros по умолчанию во многом best-effort. Подписывайся с QoS, совместимым с издателем
  (`ros2 topic info -v`), иначе сообщения не придут.
- `launch/mavros.launch` и `mavros_config.yaml` → `mavros.launch.py` и YAML для mavros ROS 2
  (`fcu_url`, `gcs_url`, `tgt_system`, список плагинов). Оставь те же аргументы launch (`fcu_conn`, `fcu_ip`,
  `fcu_sys_id`, `gcs_bridge`), где это возможно.

## Правила переноса

1. **Публичный API не меняется:** имена сервисов (`navigate`, `navigate_global`, `get_telemetry`,
   `set_position`, `set_velocity`, `set_attitude`, `set_rates`, `set_altitude`, `set_yaw`, `set_yaw_rate`,
   `land` и т. д.), типы, поля и константы в `.srv`. Сверяйся с `advertiseService` в оригинале и с README.
   Если ROS 2 запрещает имя поля (например, CamelCase), остановись и спроси.
2. Не меняй алгоритмы и логику `simple_offboard`, `vpe_publisher`, `optical_flow`. Обёртки (время, QoS,
   параметры, сервисы) переписывай, математику нет. Найденные баги пиши в `NOTES.md` и не исправляй.
3. Время и tf: `rclcpp::Time`, `this->now()`, `tf2_ros::Buffer(get_clock())`, заголовки `.hpp`. Учти `use_sim_time`.
4. Не оставляй мёртвый ROS 1 код под `#ifdef`.
5. **Лицензионные шапки не менять**, в том числе упоминания сторонних источников. `LICENSE` не трогать,
   в `package.xml` оставь оригинальных `<author>` и `<maintainer>`.
6. Блокирующие вызовы сервисов и `waitForMessage` в колбэках ROS 2 приводят к дедлоку. Сначала разбери,
   где они есть, и предложи схему (отдельный callback group, асинхронные клиенты) в плане.

## Этапы (один этап на одну сессию, между ними `/clear`)

После каждого шага: `colcon build`, затем коммит. Не переходи дальше, пока сборка не проходит.

- **A. Скелет и интерфейсы.** `package.xml`, `CMakeLists.txt`, `msg/State.msg`, все `srv/*.srv`,
  Python-пакет `clover` (`__init__.py`: `long_callback` на rclpy).
- **B. `simple_offboard.cpp`.** Только он. Начни с плана: карта сервисов, клиентов mavros, подписок, потоков.
- **C. Мелкие узлы:** `shell.cpp`, `camera_markers.cpp`, `vpe_publisher.cpp`, `rc.cpp`, `optical_flow.cpp`.
- **D. LED:** интерфейсы `led_msgs` + `led.cpp`.
- **E. `selfcheck.py`.**
- **F. Launch и конфиги:** `clover.launch.py`, `mavros.launch.py`, `aruco.launch.py`, `main_camera.launch.py`,
  `led.launch.py`, параметры mavros.
- **G. Тесты и примеры:** `test/` на launch_testing (с mock-публикаторами топиков mavros), `examples/*.py` на rclpy,
  README с отличиями от ROS 1.

## Как работать

- Начни с plan mode и покажи план этапа, код не пиши до моего ответа.
- Читай только файлы текущего этапа. Не запускай рекурсивный grep по всему репозиторию.
- Перед удалением файлов (`cfg/`, `nodelet_plugins.xml`, старые `.launch`, `.test`, `setup.py`) спрашивай.
- Если сборка падает больше 3 раз подряд на одной проблеме, остановись и покажи полный лог и гипотезу.
- Коммитить можно только то, что собирается. Идентичность git не меняй.

## Критерий готовности (этапы A–G)

- `colcon build --packages-select clover` без ошибок и без deprecation-предупреждений.
- `colcon test --packages-select clover` проходит.
- `ros2 service list -t` и `ros2 param list` для запущенных узлов соответствуют оригиналу (с учётом
  замены `/` на `.` в параметрах), список расхождений в `NOTES.md`.
- В `README.md` описаны запуск и отличия от ROS 1, отдельно перечислено, что не работает без драйверов
  ленты и дальномера.