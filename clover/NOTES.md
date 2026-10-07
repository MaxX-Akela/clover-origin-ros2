# Заметки по порту `clover` на ROS 2 Jazzy

Здесь записаны расхождения с оригиналом (ROS 1 Noetic), особенности окружения и найденные в оригинале
странности, которые при порте намеренно не исправлялись.

Проверено только сборкой и прогонами с mock-узлами. На железе и в симуляции ничего не проверялось.

## Окружение

- **Геоид для mavros.** `mavros_node` падает при старте, если нет файла
  `/usr/share/GeographicLib/geoids/egm96-5.pgm`. Нужно один раз выполнить
  `sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh`. То же самое нужно делать при сборке
  образа для Raspberry Pi 5, иначе mavros там не запустится. Без sudo можно скачать `egm96-5` в свой каталог
  и указать его через переменную `GEOGRAPHICLIB_DATA`.
- В текущем окружении (WSL) геоид лежит в `~/.local/share/GeographicLib/geoids`, mavros запускается с
  `GEOGRAPHICLIB_DATA=~/.local/share/GeographicLib`. В `~/.bashrc` переменная не прописана.
- Проверено с `ros-jazzy-mavros` 2.15.1.
- Пакета `ros2component` нет (`ros2 component load` недоступна), компонент в дымовом прогоне загружается
  сервисом контейнера `load_node`.

## Сборка пакета

- **`ament_python_install_package(clover ...)` не используется.** `rosidl_generate_interfaces` сам
  устанавливает Python-пакет `clover` (`clover.msg`, `clover.srv`) этим же макросом, повторный вызов
  конфликтует. Поэтому `src/clover/__init__.py` ставится поверх сгенерированного пустого `__init__.py`
  через `install(FILES ...)`. Импорты `from clover import long_callback` и `from clover import srv`
  работают, как в ROS 1.
- `long_callback`: поток обработки проверяет `rclpy.ok()` после получения сообщения, а не до, чтобы
  декоратор можно было применять до `rclpy.init()`.
- Файлы `.msg` и `.srv` не менялись, кроме окончаний строк (CRLF → LF).

## mavros для ROS 2 (сверено с запущенным `mavros_node`)

- Имена сервисов и топиков, которые использует `simple_offboard`, совпадают с ROS 1:
  `mavros/set_mode`, `mavros/cmd/arming`, `mavros/state`, `mavros/local_position/pose`,
  `mavros/local_position/velocity_body`, `mavros/global_position/global`, `mavros/battery`,
  `mavros/statustext/recv`, `mavros/manual_control/control`, `mavros/altitude`,
  `mavros/setpoint_position/local`, `mavros/setpoint_raw/local`, `mavros/setpoint_raw/attitude`,
  `mavros/setpoint_attitude/thrust`.
- **QoS издателей mavros:** `state` — reliable + transient local; `manual_control/control` — reliable;
  `local_position/*`, `global_position/global`, `battery`, `statustext/recv`, `altitude`, `imu/data` —
  best effort. Подписчики mavros на `setpoint_position/local` и `setpoint_raw/*` — best effort,
  на `setpoint_attitude/*`, `vision_pose/*`, `rc/override` — reliable.
- **`mavros/setpoint_attitude/attitude` по умолчанию отсутствует.** Топик появляется только при
  `setpoint_attitude: use_quaternion: true` (в оригинальном `mavros_config.yaml` так и стоит). Пока
  параметры mavros не перенесены (этап F), `set_attitude` отвечает успехом, но его сообщения никто
  не получает. **На этапе F выяснилось, что в mavros 2.15.1 включить `use_quaternion` нельзя вообще**,
  см. раздел «Launch и конфиги».
- **Параметры плагинов** принадлежат отдельным узлам (`/mavros/local_position`, `/mavros/sys`, ...),
  вложенность записывается через точку: `tf.frame_id`, `tf.child_frame_id`.
- Сервиса `mavros/param/get` нет, есть `mavros/param/set` (`mavros_msgs/srv/ParamSetV2`) и стандартные
  сервисы параметров узла `/mavros/param`. Понадобится для `selfcheck.py` (этап E).
- **Этап C, сверено с запущенным `mavros_node` без автопилота.** Совпадают с ROS 1 по имени и типу:
  `mavros/vision_pose/pose` (`geometry_msgs/msg/PoseStamped`), `mavros/vision_pose/pose_cov`
  (`PoseWithCovarianceStamped`), `mavros/px4flow/raw/send` (`mavros_msgs/msg/OpticalFlowRad`),
  `mavros/manual_control/send` (`mavros_msgs/msg/ManualControl`), `mavros/state`,
  `mavros/local_position/pose`. Подписчики mavros на `vision_pose/*`, `px4flow/raw/send` и
  `manual_control/send` — reliable, volatile.
- **Топика `mavlink/to` (и `mavlink/from`) в mavros 2.15 нет.** Вместо них узел `mavros_router`
  обменивается с узлом UAS через `/uas1/mavlink_sink` (UAS → роутер, далее в FCU и GCS) и
  `/uas1/mavlink_source` (роутер → UAS). Тип тот же, `mavros_msgs/msg/Mavlink`, QoS с обеих сторон
  best effort, volatile. Префикс `/uas1` — параметр `uas_url` узла `/mavros`, он абсолютный и зависит от
  `tgt_system`. Этим топиком пользуется `rc` (фиктивный heartbeat GCS): имя вынесено в параметр
  `mavlink_topic`, см. раздел `rc`.
- У узла `/mavros/local_position` есть и `frame_id`, и `tf.frame_id` (оба `map` по умолчанию),
  `tf.child_frame_id` — `base_link`.
- `mavros/distance_sensor/*`: проверено на этапе F, см. раздел «Launch и конфиги».

## `simple_offboard`

Алгоритмы и математика не менялись. Изменены только обёртки.

### Расхождения с ROS 1

- **Потоки.** Узел работает в `MultiThreadedExecutor`, всё состояние защищено одним мьютексом. Там, где
  в ROS 1 вызывался `ros::spinOnce()` (ожидание трансформации, OFFBOARD, арминга, AUTO.LAND), мьютекс
  отпускается, и остальные колбэки выполняются. Правила записаны комментарием в начале файла.
- **Ожидание ответа mavros тоже отпускает мьютекс.** В ROS 1 `set_mode.call()` и `arming.call()`
  блокировали весь узел. Теперь во время вызова продолжают публиковаться setpoint-ы и обрабатывается
  телеметрия, а параллельный вызов сервиса получает `Busy`.
- **Таймаут вызовов mavros.** В ROS 1 ожидание ответа `set_mode` и `arming` было бесконечным. Теперь оно
  ограничено уже существующими параметрами: `offboard_timeout`, `arming_timeout`, `land_timeout`.
  При превышении возвращается та же ошибка, что и при недоступном сервисе
  (`Error calling set_mode service`, `Error calling arming service`, `Can't call set_mode service`).
- **Кадры `local_frame` и `fcu_frame`.** В ROS 1 читались с сервера параметров
  (`mavros/local_position/tf/frame_id`, `.../child_frame_id`). Теперь при старте читаются у узла
  `<mavros>/local_position` (`tf.frame_id`, `tf.child_frame_id`). Если узел не ответил за 5 секунд,
  берутся `map` и `base_link` с предупреждением. **Новые параметры** `local_frame` и `fcu_frame`
  (по умолчанию пустые) позволяют задать кадры явно и не ждать mavros.
- **`reference_frames`.** Словарь задаётся параметрами `reference_frames.<кадр>: <опорный кадр>`.
  Имя кадра с точкой так задать нельзя.
- **Параметр `range_topic`** объявляется только при `terrain_frame_mode: range` (в ROS 1 он тоже читался
  только в этом режиме).
- **QoS.** Подписки на телеметрию и дальномер — best effort, volatile, глубина 1 (совместимо с любым
  издателем). Закэшированное сообщение `mavros/state` при старте не приходит, узел ждёт следующего.
  `simple_offboard/state` публикуется как transient local (аналог latched).
- **Таймер setpoint-ов** работает по времени ROS (учитывает `use_sim_time`), как `ros::Timer`.
  Повторный запуск уже работающего таймера ничего не делает, как `ros::Timer::start()`.
- Локальная функция `hypot` переименована в `hypot3` (конфликт с `std::hypot` в C++17), формула та же.
- Узел TF-слушателя не создаётся отдельно, слушатель работает в своём потоке внутри узла.

### Найдено в оригинале, не исправлялось

- Параметр `auto_release` читается, но нигде не используется.
- `rates_pub` (`mavros/setpoint_attitude/cmd_vel`) создаётся, но ничего не публикует.
- Комментарии `// handle pitch rate` и `// handle roll rate` перепутаны местами (код верный).
- Кадры `terrain` и `navigate_target` публикуются через `StaticTransformBroadcaster`, хотя меняются
  со временем.
- В `getTelemetry` результат `waitTransform` игнорируется, ошибка всплывает уже из `lookupTransform`.
- `nav_from_sp_flag` сбрасывается сообщениями `state` с режимом не OFFBOARD, пока `navigate` с
  `auto_arm` ждёт переключения. Поэтому первый `navigate` после взлёта с `auto_arm` стартует от текущей
  позиции, а не от setpoint-а. Поведение перенесено как есть.
- `navigate` с несуществующим `frame_id` сообщает `Can't transform from X to map`: проверка
  `X → X` всегда проходит.

## Мелкие узлы (этап C)

Проверено сборкой с `-Wall -Wextra` и дымовыми прогонами с mock-издателями (тогда `test/smoke_<узел>.py`, с этапа G — `test/test_<узел>.py`),
плюс совпадение издателей и подписчиков с запущенным `mavros_node` без автопилота. С камерой, реальными
кадрами и автопилотом ничего не проверялось.

### Общее

- **Кадры.** `vpe_publisher` и `optical_flow` получают кадры через `src/mavros_frames.hpp` по той же
  схеме, что `simple_offboard`: параметры `local_frame`/`fcu_frame` (**новые**, по умолчанию пустые) →
  узел `mavros/local_position` (`tf.frame_id`, `tf.child_frame_id`, ожидание до 5 с) → `map`/`base_link`
  с предупреждением. `simple_offboard.cpp` переведён на общий заголовок на этапе G.
- **QoS.** Подписки на чужие данные (`mavros/*`, `camera_info`, `image_raw`, `~/pose`, `~/pose_cov`) —
  best effort, volatile, глубина 1. Публикации в mavros — reliable, глубина 1. Latched-топики
  (`camera_markers`, `state_latched`) — reliable + transient local.
- Приватные имена (`~pose`, `~vpe`, `~reset`, `~shift`, `~debug`, ...) остались приватными (`~/...`).
- Флаги `-Wall -Wextra` с этапа G заданы для всего пакета.

### `shell`

- Сервис `exec` выполняется в своей группе колбэков: команды идут по очереди, как в ROS 1, сервисы
  параметров узла при этом отвечают.
- Найдено в оригинале, не исправлялось: параметр `timeout` читается, но не применяется
  (`// TODO: handle timeout`), константа `CODE_TIMEOUT` не используется; в `code` возвращается сырой
  статус `pclose()` (для `exit 3` это 768), а не код возврата; переменная `result` не используется.

### `camera_markers`

- Вместо `waitForMessage` — подписка на `camera_info`: по первому сообщению узел создаёт издателя
  `camera_markers`, публикует маркеры и отписывается. До первого `camera_info` топика `camera_markers` нет,
  как и в ROS 1.
- `ros::init_options::AnonymousName` в ROS 2 нет, имя узла — `camera_markers`. При нескольких камерах
  имя или пространство имён задаётся в launch (этап F).

### `vpe_publisher`

- Один поток исполнителя, колбэки идут по очереди, как в ROS 1, мьютекса нет. TF-слушатель работает
  в своём потоке, поэтому ожидание трансформации (0.02 с) в колбэке работает.
- Локальный кадр в оригинале читался из `mavros/local_position/frame_id` (без `tf/`), а кадр FCU из
  `mavros/local_position/tf/child_frame_id`. Теперь оба берутся по общей схеме (`tf.frame_id`,
  `tf.child_frame_id`). В mavros 2.15 `frame_id` и `tf.frame_id` по умолчанию равны `map`.
- `force_init_timeout` и `force_init_duration` объявляются только при `force_init` (или `publish_zero`),
  в оригинале они тоже читались только тогда.
- `tf::getYaw` заменён тем же расчётом через `tf2::Matrix3x3::getRPY`, `tf::Transform` — на
  `tf2::Transform`. Формулы те же.
- Найдено в оригинале, не исправлялось: кадр смещения публикуется через `StaticTransformBroadcaster`,
  хотя пересчитывается при `reset` и по `offset_timeout`.

### `rc`

- **`mavlink/to` → параметр `mavlink_topic`** (новый, по умолчанию `/uas1/mavlink_sink`, best effort).
  При `tgt_system` ≠ 1 или своём `uas_url` у mavros параметр нужно менять.
- Поток fake GCS с `ros::Rate(1)` заменён таймером 1 Гц (в оригинале стоял `TODO: use timer`).
  Поток сокета остался отдельным потоком, но перед `recvfrom` добавлен `poll()` на 0.5 с, чтобы поток
  завершался вместе с узлом (в оригинале потоки были `detach`). Сокет при выходе закрывается.
- `last_manual_control` защищён мьютексом (в оригинале пишется и читается из разных потоков без защиты).
- Таймер таймаута `state` — обычный таймер на 3 с, который создаётся остановленным, перезапускается
  каждым сообщением `mavros/state` и останавливает сам себя при срабатывании.
- Издатели создаются в конструкторе, а не в потоках. Параметр `port` (35602) сохранён.
- Закэшированное сообщение `mavros/state` при старте не приходит (подписка volatile, как в
  `simple_offboard`), до первого сообщения в `state_latched` лежит пустое состояние.
- Найдено в оригинале, не исправлялось: при ошибке `recvfrom()` и при неверном размере пакета содержимое
  буфера всё равно разбирается и публикуется; после ошибки `bind()` узел пишет «UDP RC initialized»; в шапке
  файла топик назван `latched_state`, в коде `state_latched`; порядок байт пакета не учитывается.

### `optical_flow`

- Компонент `clover::OpticalFlow` (библиотека `optical_flow_component`), исполняемый файл `optical_flow`
  запускает его отдельным процессом. В ROS 1 nodelet назывался `clover/optical_flow`.
- `dynamic_reconfigure` (`cfg/Flow.cfg`) заменён параметром `enabled` (по умолчанию `true`), меняется
  на лету; при выключении сбрасывается предыдущий кадр, как раньше.
- Параметр `image_transport` (по умолчанию `raw`) объявлен явно, в ROS 1 он читался неявно.
- Всё состояние под одним мьютексом (контейнер может быть многопоточным). TF-слушатель в своём потоке.
- Чтение кадров идёт в конструкторе. Если `local_frame`/`fcu_frame` не заданы, а mavros не отвечает,
  загрузка компонента в контейнер задерживается до 5 с. В launch кадры лучше задавать явно.
- Из тела `flow` убрана неиспользуемая метка `publish_debug:` (иначе `-Wunused-label`), `goto` на неё
  в оригинале не было. Остальной расчёт не менялся.
- Найдено в оригинале, не исправлялось: `drawFlow` рисует по изображению, которое `toCvShare` может
  разделять с входным сообщением (при `mono8` на входе меняются данные чужого сообщения); ROI считается
  один раз и не пересчитывается при смене разрешения или `camera_info`; если трансформация в кадр FCU
  недоступна, колбэк выходит до обновления предыдущего кадра и до публикации отладочного изображения.
- `cfg/Flow.cfg` и `nodelet_plugins.xml` удалены, они больше не используются.

## LED (этап D)

Проверено сборкой с `-Wall -Wextra` и дымовым прогоном с mock-драйвером (тогда `test/smoke_led.py`, с этапа G — `test/test_led.py`)
(сервис `led/set_leds`, топик `led/state`), mock `mavros/state`, `mavros/battery` и сообщениями в
`/rosout`. С настоящей лентой и автопилотом ничего не проверялось, драйвера ленты нет.

### Пакет `led_msgs`

- Новый пакет в корне репозитория. `msg/LEDState.msg`, `msg/LEDStateArray.msg`, `srv/SetLED.srv`,
  `srv/SetLEDs.srv` скопированы без изменений из `CopterExpress/ros_led` (коммит `4417455`, MIT,
  Copyright (c) 2019 Copter Express). `LICENSE` — дословная копия из корня `ros_led` (в оригинале
  в `led_msgs/` своего файла нет), вместе с примечанием про `ws281x/vendor/rpi_ws281x`, которого здесь нет.
- Версия, описание, автор и первый maintainer в `package.xml` взяты из оригинала. Зависимость `std_msgs`
  не перенесена (не используется). `sim.py` и `CHANGELOG.rst` из оригинального `led_msgs` не копировались.
- Заголовок сервиса `SetLEDs` в C++ называется `led_msgs/srv/set_le_ds.hpp` (так rosidl переводит имя).

### Узел `led`: расхождения с ROS 1

- **Имена** не менялись: `<led>/set_leds`, `<led>/state`, `<led>/set_effect` (параметр `led`, по умолчанию
  `led`), `mavros/state`, `mavros/battery`. Имя узла `led`.
- **Журнал: `/rosout` вместо `/rosout_agg`**, тип `rcl_interfaces/msg/Log`. Событие `error` — это
  `level >= 40` (ERROR и FATAL; в ROS 1 было `>= 8`, то есть те же два уровня). Подписка reliable,
  volatile, глубина 1: ошибки, случившиеся до старта узла, не приходят.
- **`set_leds` вызывается асинхронно, ответа никто не ждёт.** В ROS 1 каждый кадр блокировал узел до ответа
  драйвера. Теперь, пока предыдущий запрос не завершён, новые кадры копятся в одном ожидающем запросе
  **со слиянием по индексу светодиода** (побеждает последний цвет) и уходят сразу после ответа. Итоговое
  состояние ленты не теряется, но при медленном драйвере промежуточные кадры склеиваются (в прогоне с
  задержкой драйвера 0.12 с wipe на 10 светодиодов ушёл за 5 вызовов). Если драйвер успевает, вызовы те же,
  что в ROS 1.
- **Таймаут ответа `set_leds` — 1 с** (константа в коде, в ROS 1 ожидание было бесконечным). По таймауту
  и при недоступном сервисе пишется `Error calling set_leds service`, ожидающие кадры при недоступном
  сервисе отбрасываются. Таймаут проверяется при следующем кадре.
- **Таймер эффектов пересоздаётся** при каждой смене эффекта (у таймера rclcpp период фиксирован, в ROS 1
  был `setPeriod(..., true)`). Колбэк старого таймера, успевший встать в очередь, отбрасывается по номеру
  поколения. Первый кадр по-прежнему рисуется сразу из `set_effect`.
- **Потоки.** `MultiThreadedExecutor`, всё состояние под одним мьютексом, правила в начале файла.
  Во время `flash` мьютекс отпускается на время пауз: кадры таймера в это время **пропускаются**
  (в ROS 1 задерживались), остальные вызовы `set_effect` и события ждут окончания `flash`, как в
  однопоточном узле.
- **Старт.** Узел ждёт сервис `set_leds` и первое сообщение `state` в `main`, а не в колбэке. До этого
  сервиса `set_effect` нет, как в ROS 1.
- **Таблица событий `notify`.** Параметры `notify.<событие>.effect|r|g|b` для событий `startup`,
  `connected`, `disconnected`, `armed`, `disarmed`, `acro`, `stabilized`, `altctl`, `posctl`, `offboard`,
  `low_battery`, `error` объявлены **без значения**. Событие срабатывает, если задан хотя бы один из
  четырёх параметров (в ROS 1 — если он есть на сервере параметров). Значения читаются при каждом событии,
  `ros2 param set` действует на лету. Параметры для других событий (остальные режимы полёта, например
  `notify.manual.r`) подхватываются только из переданных при запуске; добавить новое событие на лету
  нельзя. Таблицы цветов по умолчанию в узле нет, как и в ROS 1: она была в `led.launch` и переедет в
  YAML на этапе F. Вложенный YAML `notify: startup: { r: 255, ... }` загружается.
- `r`, `g`, `b` в таблице должны быть целыми, `effect` — строкой: для известных событий параметр другого
  типа не даст узлу запуститься.
- **QoS.** `mavros/state` — reliable + transient local (узел получает закэшированное состояние и сразу
  сообщает `connected`, если FCU уже подключён); `mavros/battery` — best effort; `<led>/state` —
  reliable + transient local. **Требование к будущему драйверу ленты:** публиковать `state` как
  transient local (аналог latched в `ws281x`), иначе QoS несовместимы и узел `led` не стартует.
- `ros::names::validate` заменён локальной функцией с тем же правилом, `boost::to_lower_copy` — на
  `std::transform`. Boost больше не нужен.

### Найдено в оригинале, не исправлялось

- После `flash` заливка восстанавливается циклом, который вызывает `fill` `led_count` раз, и ещё одним
  `callSetLeds` (в ROS 1 это `led_count + 1` одинаковых вызовов сервиса; теперь они сливаются).
- `fill` не останавливает таймер предыдущего эффекта: после `blink` или `rainbow` таймер продолжает
  срабатывать вхолостую.
- `fade` читает `start_state.leds[i]` без проверки размера; если число светодиодов выросло после
  запоминания состояния, это выход за границы массива.
- `wipe` делит на `led_count`; при нуле светодиодов период бесконечен.
- В `handleMavrosState` цепочка `else if`: при одновременной смене `connected`, `armed` и режима
  сообщается только первое изменение.
- В `notify` один общий throttle (5 с) на сообщение `led: notify ...` для всех событий.
- Собственная ошибка узла (`Unknown effect`) попадает в журнал и сама вызывает событие `error`.
- `low_battery` вызывается на каждую ячейку ниже порога, то есть несколько раз на одно сообщение.

## `selfcheck` (этап E)

Запуск: `ros2 run clover selfcheck` (`--ros-args -p parallel:=true -p time:=true`). Проверено сборкой и
дымовым прогоном `test/smoke_selfcheck.py` в последовательном и параллельном режимах: (A) с запущенным
`mavros_node` без автопилота и без других данных — все 19 проверок отрабатывают без исключений и выдают
ожидаемые failure/info; (B) с mock-издателями, mock-сервисом параметров PX4, mock-оболочкой nsh и
узлами-заглушками — ни одного failure. С автопилотом, камерой, на Raspberry Pi и на образе Clover
(`/etc/clover_version`) ничего не проверялось: ветки `clover.service`, `Network`, `Boot duration` и
`vcgencmd` в этом окружении не исполняются дальше первой проверки.

### Зависимости

- **`pymavlink`**: apt-пакета в Ubuntu 24.04 нет, правила rosdep нет, в `ros-jazzy-mavlink` модуля нет
  (у mavros эта зависимость в `package.xml` закомментирована). Ставится через pip (`pip install pymavlink`,
  здесь 2.4.49 в `~/.local`), на образе это нужно предусмотреть. В `package.xml` не записан.
- `from mavros import mavlink` есть в `ros-jazzy-mavros`.
- `tf.transformations` не используется, `ros-jazzy-tf-transformations` не нужен: углы Эйлера считает
  локальная `euler_from_quaternion` (только `axes='rzyx'`), сверена со `scipy` на 20000 случайных
  кватернионов (расхождение до 5e-14 рад).
- В `package.xml` добавлены `exec_depend`: `aruco_pose`, `diagnostic_msgs`, `python3-numpy`.

### Расхождения с ROS 1

- **Список и порядок проверок, все тексты `failure`/`info` совпадают с оригиналом**, кроме перечисленного
  ниже (сверено по AST: 19 из 19 имён, из 134 сообщений нет двух, добавлено семь).
- **Убраны** `no ROS_HOSTNAME is set` и `not found %s in /etc/hosts, ...`: в ROS 2 нет `ROS_HOSTNAME`.
  Проверка `Network` осталась; на образе Clover она выводит info `ROS_DOMAIN_ID = ...`,
  `ROS_AUTOMATIC_DISCOVERY_RANGE = ...`, `ROS_LOCALHOST_ONLY = ...` для заданных переменных.
- **Новые сообщения** (вместо исключений): `can't read parameters of <узел> node` (узел не ответил на
  запрос параметров), `could not call systemctl: ...`, `could not call systemd-analyze: ...`,
  `could not parse systemd-analyze output`, `vcgencmd returned <код>`, `could not parse vcgencmd output: ...`.
  Отсутствие `pgrep` считается как «процесс не запущен».
- **Поиск процессов заменён графом ROS.** `aruco_detect`, `aruco_map`, `optical_flow`, `vpe_publisher`
  ищутся как узлы в пространстве имён `selfcheck` (в ROS 2 они могут жить в `component_container`).
  Тексты `... is not running` те же. `pgrep -x px4` (признак SITL) оставлен.
- **Параметры чужих узлов** читаются сервисом `<узел>/get_parameters` (таймаут 1 с):
  `aruco_detect` (`length`, `known_vertical`, `flip_vertical`), `aruco_map` (`known_vertical`,
  `flip_vertical`), `optical_flow` (`disable_on_vpe`). Если узел не ответил, берётся то же значение
  по умолчанию, что в оригинале, а для `length` — `aruco_detect/length parameter is not set`.
- **`fcu_url`** в mavros 2.15 — параметр узла **`/mavros_node`** (у узла `/mavros` его нет, там `uas_url`).
  Читается у `mavros_node` в пространстве имён `selfcheck`. Если на этапе F узел в launch получит другое
  имя, поправить константу `MAVROS_NODE`; без неё выводится `fcu_url = ?`.
- **Параметры PX4**: вместо `mavros/param/get` — `mavros/param/get_parameters`
  (`rcl_interfaces/srv/GetParameters`). Без FCU mavros отвечает `PARAMETER_NOT_SET` за ~0.1 с, это
  `unable to retrieve PX4 parameter ...`, как раньше. Нет сервиса (ждём 1 с) —
  `<имя>: service [/mavros/param/get_parameters] unavailable`. Сохранено поведение `ParamGet`: целое ≠ 0
  возвращается как int, иначе float (целый ноль — `0.0`).
- **MAVLink-оболочка.** `mavlink/to` и `mavlink/from` → параметры `mavlink_topic`
  (`/uas1/mavlink_sink`, как в `rc`) и **`mavlink_from_topic`** (`/uas1/mavlink_source`, роутер → UAS;
  издатель `mavros_router`, best effort, volatile — сверено с запущенным `mavros_node`). При `tgt_system` ≠ 1
  менять оба. `pymavlink` с диалектом по умолчанию (v1) разбирает кадры MAVLink 2 и сам отправляет кадры v1
  (проверено mock-оболочкой с кадрами v2). Проходит ли `SERIAL_CONTROL` через роутер mavros до FCU и
  обратно, без автопилота не проверить.
- **Диагностика `mavros: Time Sync`.** Без FCU mavros такого статуса в `/diagnostics` не публикует
  (есть `mavros: Heartbeat`, `System`, `Battery`, `GPS`, ...), поэтому имя статуса и ключ
  `Estimated time offset (s)` для mavros 2.15 **не подтверждены**, проверено только на mock.
- **Ожидание сообщений.** `rclpy.wait_for_message` не используется: он ждёт подписку в своём WaitSet, а
  фоновый executor может забрать сообщение раньше. Своя функция: подписка с колбэком и событием, после
  ожидания подписка удаляется. То же для `/diagnostics`. Подписка на `mavlink_from_topic` одна на весь запуск.
- **QoS.** `mavros/state` и `aruco_map/visualization` — reliable + transient local (как у издателей),
  всё остальное — best effort, volatile, глубина 1.
- **Сервисы `navigate`, `get_telemetry`, `land`** ищутся в графе по имени, клиенты не создаются.
- **Потоки.** `rclpy.init`, узел и подписки создаются в `main()`, а не при импорте. Узел крутит
  `MultiThreadedExecutor` в фоновом потоке, колбэки только сохраняют значение и ставят событие. Проверки
  идут в главном потоке или (при `parallel`) каждая в своём потоке. Создание и удаление подписок и
  клиентов — под одним замком. Потоки проверок стали daemon (Ctrl+C завершает программу).
- **Вывод** идёт через логгер узла с форматом `{message}` (`RCUTILS_CONSOLE_OUTPUT_FORMAT`), то есть
  весь в stderr (в ROS 1 info шёл в stdout), и в `/rosout`. Цвета включаются по `sys.stderr.isatty()`.
- **`clover.service`.** Имя unit-а и путь `/tmp/clover.err` оставлены, **оба зависят от образа**, которого
  для ROS 2 пока нет. Регулярное выражение разбора лога переписано под формат консоли ROS 2
  (`[ERROR] [время] [узел]: текст`, `WARN` без пробела); вывод в прежнем виде `ERROR: текст`.
- **`CPU usage`.** В белом списке `nodelet` → `component_conta` (так `top` показывает
  `component_container`), `selfcheck.py` → `selfcheck`.
- Пауза 0.5 с перед проверками сохранена; в прогонах её хватало на обнаружение узлов.
- `install(PROGRAMS ... RENAME selfcheck)` копирует файл и при `--symlink-install`: после правки
  `selfcheck.py` нужна пересборка.
- Окончания строк файла: CRLF → LF (иначе не работает шебанг).

### Найдено в оригинале, не исправлялось

- **В параллельном режиме `FCU` и `Preflight status` делят буфер оболочки** (`mavlink_recv`,
  `recv_event`) без блокировки. В прогоне с mock это воспроизвелось: `FCU` не получил вывод `ver all`
  и вывел `not Clover PX4 firmware, ...` вместо версии.
- Разность рысканья в `Vision position estimate` считается в радианах, а нормируется как градусы
  (`(yawdiff + 180) % 360 - 180`) и потом ещё раз переводится в градусы.
- Все три сообщения об угловых скоростях называют их `pitch rate`.
- В `Optical flow` бит `1 << 1` в `LPE_FUSION` проверяется дважды (второе сообщение про gyro compensation).
- Если `get_param` вернул `None` (нет параметра или сервиса), `int(get_param('LPE_FUSION'))`,
  `int(get_param('EKF2_AID_MASK'))` и деление на `n_cells` дают исключение (`exception occurred`).
- `rospy.get_param('aruco_detect/length', '?')` с `%g`: при отсутствии параметра было исключение
  `TypeError`, а `except KeyError` не срабатывал. В порте `KeyError` достижим.
- В тексте `calibration height doesn't match image height (%d != %d))` лишняя скобка.
- `ff` возвращает `None` для значений не `float` и не `int`.
- `failure(error)` для строк `/tmp/clover.err`: строка с `%` ломает форматирование.
- `Boot duration` берёт только секунды: `1min 2.3s` читается как 2.3 с.
- `check_camera` принимает имя камеры, но сообщение об ориентации его не содержит.

## Launch и конфиги (этап F)

Проверено сборкой, отдельными запусками каждого launch-файла с вариантами аргументов и
`test/smoke_launch.py` (три режима: без FCU и камеры; с `mavros_node` без автопилота; с
`main_camera.launch.py`, aruco и mock-камерой). С настоящей камерой, автопилотом, лентой и дальномером
ничего не проверялось. **Шесть apt-пакетов не установлены** (`v4l2_camera`, `image_proc`, `topic_tools`,
`web_video_server`, `rosbridge_server`, `tf2_web_republisher`), всё, что от них зависит, написано по
документации и **не запускалось** (отмечено ниже).

Файлы: `launch/*.launch.py`, `config/mavros.yaml`, `config/led_notify.yaml`, `src/mavros_params.py`.
Старые `launch/*.launch` и `launch/mavros_config.yaml` удалены на этапе G.

### mavros

- **Имена `/mavros/...` даёт имя узла UAS, а не пространство имён.** Процесс `mavros_node` создаёт узлы
  `/mavros_node` (`fcu_url`, `gcs_url`, `tgt_system`, `tgt_component`), `/mavros_router` и `/mavros`
  (`plugin_allowlist`, `plugin_denylist`, `uas_url`, `fcu_protocol`), плагины — узлы `/mavros/<имя>`.
  В launch узел запускается **без `name` и без `namespace`**: `name='mavros'` переименовал бы все три узла,
  `namespace='mavros'` удвоил бы имена. `MAVROS_NODE = 'mavros_node'` в `selfcheck.py` и
  `mavros/local_position` в `mavros_frames.hpp` верны, не менялись (`selfcheck` выводит `fcu_url`, узлы
  читают кадры без предупреждения).
- **Список плагинов.** `plugin_whitelist` → `plugin_allowlist`. В mavros 2 allowlist только отменяет
  denylist, поэтому задано `plugin_denylist: ['*']` + `plugin_allowlist: [...]`; сверено по списку узлов
  `/mavros/*`. Плагин `vision_pose_estimate` теперь называется `vision_pose`. Имена узлов плагинов
  отличаются от имён плагинов: `sys_status` → `mavros/sys`, `sys_time` → `mavros/time`, `command` →
  `mavros/cmd`, `rc_io` → `mavros/rc`.
- **Параметры плагинов при запуске не применяются (ограничение mavros 2.15.1).** Узлы плагинов создаются
  с `use_global_arguments(false)` (`mavros/src/lib/plugin.cpp`), поэтому ни `--params-file`, ни `-p`, ни
  ремапы процесса `mavros_node` до них не доходят: с любым YAML `conn_timeout` остаётся 10,
  `local_position tf.send` — `false`. То же относится к штатному `px4.launch` из пакета mavros (проверено отдельно, см. «Проверка диагноза» ниже). В ветке
  `ros2` апстрима это исправлено (плагин забирает переопределения по своему полному имени), в apt
  исправления пока нет.
- **Обход: узел `mavros_params`** (`src/mavros_params.py`, новый). Читает `config/mavros.yaml` и выставляет
  параметры плагинов через их сервисы `set_parameters`, как только сервис появляется; при перезапуске
  mavros (respawn) повторяет. Проверено: `conn_timeout: 8.0`, `tf.send: true`, `config` дальномеров
  применяются. Между стартом mavros и применением параметров проходит до ~2 с. `config/mavros.yaml`
  записан в формате апстрима (`/**/<узел плагина>: ros__parameters:`), так что с исправленным mavros он
  применится и при запуске.
- **`setpoint_attitude: use_quaternion: true` включить нельзя.** При запуске параметр не доходит (см. выше),
  а на лету mavros отвечает `Invalid history policy enum value passed to QoSInitialization::from_rmw`
  (в колбэке параметра используется ссылка на локальную переменную конструктора) и остаётся **без обеих
  подписок**, `cmd_vel` и `attitude`. Поэтому `mavros_params` этот параметр пропускает (константа `SKIP`).
  Следствие: топика `mavros/setpoint_attitude/attitude` нет, **`set_attitude` на mavros 2.15.1 не
  работает** (сервис отвечает успехом, сообщения никто не получает). Нужен mavros с исправлением
  (сборка ветки `ros2` из исходников или новая версия из apt).
- **Параметры, которых в 2.15 нет** (в YAML не перенесены): `startup_px4_usb_quirk`, `gcs_quiet_mode`,
  `time.timesync_avg_alpha` (теперь `timesync_alpha_initial/final`, `timesync_beta_*`),
  `time.publish_sim_time`, `local_position tf.send_fcu`, `setpoint_attitude tf.*`,
  `setpoint_position tf.*`, блок `odometry in/out`. `conn/heartbeat_rate` и `conn/timeout` →
  `sys.heartbeat_rate`, `sys.conn_timeout`; `conn/timesync_rate`, `conn/system_time_rate` → `time.*`.
  `target_system_id` → `tgt_system`. У `vision_pose` параметры называются через косую черту
  (`tf/listen`), а не через точку, как написано в `px4_config.yaml` самого mavros.
- `!degrees`: `angular_velocity_stdev: 0.0003490659` (0.02°), `ranger_fov: 0.118682` (6.8°).
- `conn_timeout`: в оригинале 10 в YAML и 8 в launch (побеждал launch), в `config/mavros.yaml` записано 8.
- **`distance_sensor`.** Настройки — один строковый параметр `config` с YAML внутри. Топики называются по
  ключу датчика относительно `/mavros` (ключ `rangefinder` дал бы `/mavros/rangefinder`), поэтому ключи
  записаны как `distance_sensor/rangefinder` и `distance_sensor/rangefinder_sub`: имена топиков совпадают
  с ROS 1 и с тем, что ждёт `selfcheck`. Издатель и подписчик best effort.
- **Ремап `mavros/distance_sensor/rangefinder_sub` → `rangefinder/range`** аргументами процесса невозможен
  (та же причина). Вместо него `topic_tools relay` (узел `rangefinder_relay`) из `distance_sensor_remap` в
  `mavros/distance_sensor/rangefinder_sub`. **Не запускалось** (`topic_tools` не установлен); без пакета
  launch печатает предупреждение, данные дальномера в FCU не идут.
- Дубль настроек по имени `rangefinder/range` (`<rosparam param="$(arg distance_sensor_remap)">`) не
  перенесён: глобальных параметров в ROS 2 нет.
- **`gcs_host`** (новый аргумент, по умолчанию пустой) заменяет `$(env ROS_HOSTNAME)` в `udp-b` и `udp-pb`.
  Пустое значение даёт `udp-b://:14550@14550`. Отключить мост: `gcs_bridge:=false` (пустое
  `gcs_bridge:=` `ros2 launch` не принимает).
- Неизвестное значение `fcu_conn`: как в оригинале, `fcu_url` не задаётся (у mavros свой по умолчанию),
  плюс сообщение в журнале.
- `fcu_conn:=usb`: префикс `waitfile /dev/px4fmu`, `respawn` с задержкой 1 с — как в оригинале.
  `src/waitfile` устанавливается как программа (`ros2 run clover waitfile`), окончания строк CRLF → LF.
- **Узла `visualization` в `mavros_extras` для Jazzy нет** (есть только `servo_state_publisher` и
  `terrain_server`). Аргумент `viz` оставлен, при `true` выводится сообщение.
- При остановке по Ctrl+C `mavros_node` иногда завершается с кодом -2 (`process has died`), процессов
  после этого не остаётся.
- `rc`: при `use_fake_gcs: false` параметр `mavlink_topic` узлом не объявляется; launch передаёт
  `/uas<fcu_sys_id>/mavlink_sink` на случай включения. `selfcheck` запускается отдельно, при
  `fcu_sys_id` ≠ 1 его `mavlink_topic` и `mavlink_from_topic` нужно задавать вручную.

### Проверка диагноза про параметры плагинов (после этапа F)

Вопрос: правда ли, что штатный `px4.launch` тоже не применяет параметры плагинов, или причина в нашем
`config/mavros.yaml` / `mavros.launch.py`. **Ответ: штатный launch тоже не применяет, причина в mavros
2.15.1.** Проверено запуском без автопилота (`GEOGRAPHICLIB_DATA=~/.local/share/GeographicLib`).

- **Версии.** `ros-jazzy-mavros 2.15.1-1noble.20260903.022619` (extras, msgs, libmavconn — тоже 2.15.1),
  `apt-cache policy`: кандидат тот же, обновления в apt нет.
- **Опыт.** Копия `px4.launch`, в которой `config_yaml` указывает на копию `px4_config.yaml` с тремя
  правками: `sys: conn_timeout: 8.0`, `local_position: tf.send: true`,
  `setpoint_attitude: use_quaternion: true`. `node.launch` и `px4_pluginlists.yaml` — штатные.
  Запуск: `ros2 launch <копия>/px4.launch fcu_url:=udp://@127.0.0.1:14557`.
- **Командная строка процесса** (`/proc/<pid>/cmdline`), файл с правками передан:

  ```
  /opt/ros/jazzy/lib/mavros/mavros_node --ros-args -r __ns:=/ --params-file /tmp/launch_params_... (x5:
  fcu_url, gcs_url, tgt_system, tgt_component, fcu_protocol) --params-file
  /opt/ros/jazzy/share/mavros/launch/px4_pluginlists.yaml --params-file <копия>/px4_config.yaml
  ```

- **Результат** (`ros2 param get`, через 15 с после старта):

  ```
  /mavros/sys conn_timeout                 -> Double value is: 10.0     (в файле 8.0)
  /mavros/local_position tf.send           -> Boolean value is: False   (в файле true)
  /mavros/setpoint_attitude use_quaternion -> Boolean value is: False   (в файле true)
  /mavros_node fcu_url                     -> String value is: udp://@127.0.0.1:14557
  ros2 topic list | grep setpoint_attitude -> .../cmd_vel, .../thrust (топика attitude нет)
  ```

  Параметры узлов `/mavros_node` и `/mavros` (`fcu_url`, `plugin_denylist`) из тех же файлов применяются
  (в журнале `plugin_denylist pattern 'image_pub' does not match...`), параметры узлов плагинов — нет.
- **Сравнение с нашим путём.** Формат один и тот же: `/**/<узел плагина>: ros__parameters:`, имена узлов
  (`sys`, `time`, `local_position`, `setpoint_attitude`, ...) совпадают с `ros2 node list`. Способ передачи
  тоже один: `parameters=[config, params]` в `Node` превращается в те же `--params-file`:

  ```
  /opt/ros/jazzy/lib/mavros/mavros_node --ros-args --params-file .../share/clover/config/mavros.yaml
  --params-file /tmp/launch_params_...
  ```

  Отличия: штатный launch добавляет `-r __ns:=/` (пустой `namespace`), список плагинов у него в отдельном
  файле под `/**:`, у нас в словаре `params`. На применение параметров плагинов ни то, ни другое не влияет.
  В нашем запуске `conn_timeout: 8.0` и `tf.send: true` появляются только благодаря `mavros_params`.
- **Причина в исходниках.** Тег `2.15.1` (`22ae5b7`), `mavros/src/lib/plugin.cpp`, конструктор
  `Plugin::Plugin(UASPtr, const std::string & subnode, const rclcpp::NodeOptions &)`:
  `node_options.use_global_arguments(false);` — вместе с ремапами отбрасываются `--params-file` и `-p`
  (появилось в коммите `63f7392`, август 2026).
- **Исправление в апстриме.** Коммит `d9b38f6` от 2026-09-27 «mavros: re-apply process parameter sources
  to plugin sub-nodes», «Fixes #2294», вошёл в тег **`2.16.0`** (`5c68b90`, сейчас это и есть голова ветки
  `ros2`). В том же конструкторе после `use_global_arguments(false)` добавлено: `rcl_arguments_get_param_overrides`
  по глобальным аргументам контекста → `rclcpp::parameter_map_from(global_params, fqn)` для полного имени
  `<uas>/<subnode>` → `node_options.parameter_overrides(merged)`. Добавлен тест
  `mavros/test/test_plugin_params.cpp`. Наш формат `/**/sys:` под это сопоставление подходит.
- **`setpoint_attitude.cpp` между 2.15.1 и 2.16.0 не менялся.** Колбэк `use_quaternion` захватывает
  локальную `subscriber_qos` по ссылке; при объявлении параметра он вызывается внутри конструктора (переменная
  жива), так что с 2.16.0 значение из YAML при запуске должно сработать, а смена на лету останется сломанной.
  Это вывод из чтения кода, **не проверено**, пока 2.16.0 не собран.
- **Доступность 2.16.0.** В `rosdistro` (`jazzy/distribution.yaml`) для mavros уже записан релиз `2.16.0-1`,
  в основном apt-репозитории его пока нет (появится со следующей синхронизацией).
  **Обновление на этапе I (2026-10-07):** `apt-cache policy` в этом окружении (amd64 noble) показывает кандидатов
  `ros-jazzy-mavros`, `-mavros-extras`, `-mavros-msgs`, `-libmavconn` версии `2.16.0-1noble.20260927...`. Для arm64 это проверяет
  первый шаг CI (`image/scripts/check-apt-packages.sh`). Локально mavros 2.16.0 не ставился и не запускался, всё сказанное выше
  про параметры плагинов и `set_attitude` на 2.16.0 по-прежнему не проверено.
- **Что тянет за собой 2.16.0** (diff 2.15.1..2.16.0): `mavros_msgs/msg/State.msg` получил новые константы
  (`MODE_FLIX_*`), значит `mavros_msgs` нужно собирать тоже, а `clover` пересобрать поверх; у `mavros`
  новая зависимость `rcl`; `node.launch` получил `respawn`.

**Решение (этап G): mavros из исходников не собираем.** В WSL 3 ГБ памяти, при сборке диск уходит в 100 %.
Этот пункт этапа F закрыт так: `set_attitude` ждёт mavros 2.16.0 из apt (или сборки в CI), до тех пор на
2.15.1 он не работает (сервис отвечает успехом, сообщения никто не получает). Узел `mavros_params` и его
`SKIP` для `use_quaternion` остаются. После появления 2.16.0 проверить три параметра из опыта выше и топик
`mavros/setpoint_attitude/attitude`; если параметры из `config/mavros.yaml` применяются при запуске,
`mavros_params` можно убрать. С 2.16.0 ничего не проверялось.

**План сборки из исходников (не выполнялся и выполняться не будет, оставлен для справки).** Отдельное
рабочее пространство-подложка, чтобы не класть чужой код в этот репозиторий:

```
mkdir -p ~/mavros_ws/src && cd ~/mavros_ws/src
git clone --depth 1 --branch 2.16.0 https://github.com/mavlink/mavros.git   # vcs не установлен, хватает git
cd ~/mavros_ws && source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src/mavros/libmavconn src/mavros/mavros_msgs src/mavros/mavros \
  src/mavros/mavros_extras --ignore-src --rosdistro jazzy
colcon build --packages-select libmavconn mavros_msgs mavros mavros_extras \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source ~/mavros_ws/install/setup.bash   # затем пересборка clover в этом репозитории
```

`rosdep check` на этих четырёх пакетах уже прогнан, не хватает: **`ros-jazzy-angles`** (обязателен для
`mavros` и `mavros_extras`: `find_package(angles REQUIRED)`), `ros-jazzy-ament-lint-auto`,
`ros-jazzy-ament-lint-common`, `ros-jazzy-ament-cmake-google-benchmark` (только тесты, обходится
`-DBUILD_TESTING=OFF`). `angles` нужно поставить через apt либо склонировать `ros/angles` в то же `src`.
`ros-jazzy-mavlink` остаётся из apt (2026.8.8). После сборки проверить те же три параметра и топик
`mavros/setpoint_attitude/attitude`; если применяются, `mavros_params` и его `SKIP` не нужны.

### Камера

- **Контейнер `main_camera_container`** (`component_container_mt`, `thread_num: 2`) вместо
  `main_camera_nodelet_manager`. В него грузятся камера, `aruco_detect`, `aruco_map`, `optical_flow`,
  `rectify`.
- **Контейнер без respawn.** В оригинале менеджер и nodelet-ы перезапускались. В ROS 2 launch не
  загружает компоненты заново в перезапущенный контейнер, получился бы пустой контейнер. Падение любого
  компонента останавливает все компоненты до перезапуска launch.
- **`main_camera:=false`**: `clover.launch.py` сам поднимает пустой `main_camera_container`, если включены
  `optical_flow` или `aruco` (в ROS 1 загрузчики nodelet-ов без менеджера висели). Так компоненты работают
  с камерой из другого источника. `aruco.launch.py`, запущенный отдельно, ждёт контейнер (аргумент
  `container`).
- **`cv_camera` → `v4l2_camera`** (компонент `v4l2_camera::V4L2Camera`, узел `/main_camera/main_camera`:
  пространство имён `main_camera`, чтобы `image_raw`, `camera_info` и `set_camera_info` получили те же
  имена, что в ROS 1). **Не запускалось, имена параметров не сверены с установленным пакетом.**
  Соответствие: `device_path` → `video_device`, `frame_id` → `camera_frame_id`, `image_width/height` →
  `image_size: [320, 240]`, `cv_cap_prop_fps: 40` → `time_per_frame: [1, 40]`, `camera_info_url` тот же.
  **Нет аналогов:** `rate` (частота опроса), `capture_delay: 0.02` (метка времени кадра не сдвигается на
  задержку захвата, это может влиять на согласование optical flow с гироскопом), `rescale_camera_info`.
- **`rescale_camera_info`** (аргумент launch, по умолчанию `true`): launch делает временную копию
  `fisheye_cam.yaml` (640×480), умножая `fx`, `cx` в `K` и `P` на отношение ширин, `fy`, `cy` — на
  отношение высот, и передаёт её камере. Проверен только пересчёт (`K`: 332.48, 320, 333.18, 240 →
  166.24, 160, 166.59, 120). Временный файл в `/tmp` не удаляется.
- **Ожидание устройства.** `waitfile <device> true` запускается отдельным процессом, по его успешному
  завершении камера загружается в контейнер. Остальные компоненты устройства не ждут. В прогонах без
  `/dev/video0` процесс ждёт и завершается вместе с launch.
- **Кадр камеры.** `static_transform_publisher` теперь с именованными аргументами (`--x --y --z --yaw
  --pitch --roll --frame-id --child-frame-id`), числа те же. Все шесть вариантов сверены по кватерниону
  в выводе узла с расчётом по yaw/pitch/roll.
- `camera_markers`: узел `/main_camera/main_camera_markers`, `scale: 3.0`.
- `topic_tools throttle` (`image_raw` → `image_raw_throttled`, 5 Гц) и `image_proc::RectifyNode`
  (ремапы `image`, `camera_info`, `image_rect`) — **не запускались**, без пакетов launch печатает
  предупреждение.
- Разрешение 320×240 задано константами в `main_camera.launch.py` (в оригинале — в тексте launch).

### aruco, LED, прочее

- `aruco_detect`, `aruco_map`: параметры и ремапы оригинала под именами, которые объявляет порт
  `aruco_pose` (`cornerRefinementMethod`, `minMarkerPerimeterRate`, `markers.frame_id`,
  `markers.child_frame_id_prefix`, `length_override.<id>`). Сверено на запущенных компонентах для
  `placement` = `floor`, `ceiling`, `unknown`, с `aruco_map`, `aruco_vpe`, `disable`.
- `vpe_publisher`: `~/vpe` → `mavros/vision_pose/pose`, при `aruco_vpe` `~/pose_cov` → `aruco_map/pose` и
  `frame_id: aruco_map_detected`.
- `led.launch.py`: узел эффектов называется `led_effect`, как в оригинале; таблица `notify` — в
  `config/led_notify.yaml`, подключается при `led_notify:=true`. Параметры драйвера `ws281x` записаны как
  в оригинале, но драйвера нет.
- **Отсутствующие пакеты не роняют launch**: для `ws281x`, `vl53l1x`, `v4l2_camera`, `topic_tools`,
  `image_proc`, `web_video_server`, `rosbridge_server`, `tf2_web_republisher` проверяется наличие
  (`get_package_prefix`), при отсутствии выводится `WARNING: ... package is not installed`.
- **Веб-пакеты в Jazzy есть в apt** (`ros-jazzy-web-video-server` 3.1.0, `ros-jazzy-rosbridge-server`
  2.7.1, `ros-jazzy-tf2-web-republisher` 1.0.0), но не установлены, запуск не проверялся. Не сверены:
  параметры `web_video_server` (`default_stream_type`, `publish_rate`), имя файла
  `rosbridge_websocket_launch.xml`, имя исполняемого файла `tf2_web_republisher` (launch ищет
  `tf2_web_republisher`, затем `tf2_web_republisher_node`).
- `ROSCONSOLE_FORMAT` → `RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}]: {name}: {message}'`.
- Аргумент `blocks` убран (`clover_blocks` вне области). `simulator:=true` отключает камеру, `vl53l1x` и
  `ws281x`, как в оригинале.
- Кадры `local_frame`/`fcu_frame` в launch не задаются: при `fcu_conn:=none` узлы `simple_offboard`,
  `vpe_publisher`, `optical_flow` стартуют на 5 с позже и берут `map`/`base_link` с предупреждением.
- Сверка с оригиналом (запуск `fcu_conn:=udp rc:=true aruco:=true`): сервисы `navigate`,
  `navigate_global`, `get_telemetry`, `set_position`, `set_velocity`, `set_attitude`, `set_rates`,
  `set_altitude`, `set_yaw`, `set_yaw_rate`, `land`, `simple_offboard/release`, `vpe_publisher/reset`,
  `aruco_detect/set_length_override`, `mavros/set_mode`, `mavros/cmd/arming` на месте. Нет:
  `led/set_effect` (узел `led_effect` ждёт драйвер), `mavros/param/get` (см. выше), сервисов
  `dynamic_reconfigure`. Узлы: `main_camera_nodelet_manager` → `main_camera_container`,
  `main_camera` → `/main_camera/main_camera`, новые `mavros_params` и (при наличии `topic_tools`)
  `rangefinder_relay`.

### Найдено в оригинале, не исправлялось

- `optical_flow` с `roi_rad` и калибровкой без дисторсии (поле зрения меньше `roi_rad`) получает ROI за
  пределами кадра (`ROI: 33 -7 - 287 247` для 320×240) и падает с `cv::Exception`, унося весь контейнер.
  С калибровкой `fisheye_cam.yaml` ROI в пределах кадра (`97 57 - 223 183`).

## Тесты, примеры, зачистка (этап G)

Проверено сборкой с `-Wall -Wextra` (0 предупреждений) и `colcon test --packages-select clover`:
37 тестов, 0 падений, 2 пропуска (11 запусков `launch_testing`, 26 проверок внутри них). На железе и в
симуляции ничего не проверялось.

### Тесты

- Все тесты — `launch_testing` (`add_launch_test`), общие помощники в `test/clover_test_utils.py`. Тесты
  идут последовательно (`RUN_SERIAL`) в `ROS_DOMAIN_ID=87`, если переменная не задана снаружи.
- **`test/offboard.py`**: все проверки оригинала в том же порядке. В оригинальном `offboard.test` две
  статические трансформации (map→test `10 20 30`, map→test2 `100 200 300`), третья (map→map_flipped) —
  в `basic.test`; перенесены так же. Без mavros узел ждёт 5 с чтения кадров, поэтому первый вызов ждёт
  сервис до 30 с. `rospy.wait_for_message` заменён новой подпиской на каждый вызов (для
  `simple_offboard/state` — transient local).
- **`test/basic.py`**: запускает `mavros.launch.py` (`fcu_conn:=udp`, без автопилота), `simple_offboard`, `rc`,
  `shell`, `led_effect`. Отличия от оригинала: узла `visualization` нет в Jazzy; `clover_blocks` вне
  области — `test_blocks` всегда пропускается; `test_web_video_server` пропускается, если пакет не
  установлен (здесь не установлен, **тест ни разу не выполнялся**); `tf2_web_republisher`, `throttle`,
  `rectify` и менеджер nodelet-ов из запуска убраны; добавлен `test_nodes_running` (в оригинале узлы были
  `required`).
- **Геоид.** `basic.py` подставляет `GEOGRAPHICLIB_DATA=~/.local/share/GeographicLib`, если переменная
  пуста и каталог есть. Если геоида нет нигде, mavros не запускается, а `test_state` пропускается с
  командой установки в причине.
- **Из smoke-скриптов** в тесты перенесены `shell`, `camera_markers`, `rc`, `vpe_publisher`, `led`,
  `optical_flow` (`test/test_<узел>.py`), `selfcheck` на mock-данных (`test_selfcheck.py`, пропуск без
  `pymavlink`) и `clover.launch.py` с `fcu_conn:=none main_camera:=false` (`test_launch.py`). Вторые
  экземпляры `vpe_publisher` и `optical_flow` (чтение кадров из mavros) работают в своих пространствах имён
  с `test/mock_local_position.py`.
- **Остались скриптами** (нужен `mavros_node`, камера): `smoke_selfcheck.py` (режим A) и
  `smoke_launch.py --mavros/--camera`. `test_selfcheck.py` и `test_launch.py` импортируют из них mock-и и
  списки.
- В `test_selfcheck.py` в параллельном режиме отчёты `FCU` и `Preflight status` не сравниваются (гонка
  оригинала, см. выше).
- Запрос, отправленный клиентом rclpy сразу после появления сервиса в графе, может потеряться; помощник
  тестов и `service_proxy` перед каждым вызовом ждут `wait_for_service`.
- Отдельного smoke-скрипта для `simple_offboard` в репозитории не было; после перевода на
  `mavros_frames.hpp` узел проверен тестами `offboard.py`, `basic.py`, `examples.py`, `test_launch.py`.

### Примеры и `service_proxy`

- **Новый публичный объект: `clover.service_proxy(name, srv_type, timeout=None, wait_for_service=5.0)`**
  в `src/clover/__init__.py`, замена `rospy.ServiceProxy`. Возвращает функцию: синхронный вызов, поля
  запроса — позиционные или именованные аргументы, целые для полей `float` приводятся к `float` (rclpy их
  не принимает). Сервис недоступен за `wait_for_service` секунд — `RuntimeError`
  (`service [/navigate] unavailable`), нет ответа за `timeout` — `TimeoutError`; `timeout=None` ждёт
  бесконечно, как ROS 1. Узел `clover_service_proxy_<pid>` и его executor создаются при первом вызове
  `service_proxy` под замком, `rclpy.init()` вызывается, только если ещё не вызван. Остановка — через
  `atexit` (без неё процесс завершался с `terminate called without an active exception`).
- Имена сервисов разрешаются относительно узла прокси (корневое пространство имён или `__ns` процесса).
- **Примеры:** `rospy.ServiceProxy` → `service_proxy`, `rospy.init_node` → `rclpy.init()` (плюс
  `rclpy.create_node` там, где есть подписки), **`rospy.sleep` → `time.sleep`** (время стены, а не ROS:
  при `use_sim_time` с коэффициентом реального времени ≠ 1 паузы отличаются от ROS 1; в плане было
  ожидание по часам узла, но у узла, который никто не крутит, время симуляции не идёт).
- Подписки на изображение и дальномер в примерах — `qos_profile_sensor_data` (совместимо и с reliable, и
  с best effort издателем). Приватные топики: `~center` → `~/center` и т. д.
- `red_circle.py`: колбэки `rospy` шли в фоновых потоках до `rospy.spin()`; теперь узел крутится в фоновом
  потоке, главный ждёт Enter. Методы `image_geometry` переименованы в snake_case (`from_camera_info`,
  `rectify_point`, `project_pixel_to_3d_ray`; сверено по исходникам 4.1.0, пакет не установлен, **пример
  не запускался**).
- `test/examples.py`: компиляция всех девяти примеров; `get_telemetry`, `navigate_wait`, `leds`, `camera`
  запускаются с узлами пакета и mock-ами (FCU, который мгновенно «прилетает» в setpoint; драйвер ленты;
  синтетический кадр). `flight`, `flight_marker`, `gps`, `subscriber`, `red_circle` не запускались.
- Примеры устанавливаются в `share/clover/examples`.

### Зачистка

- grep по `rospy|roscpp|nodelet|catkin|dynamic_reconfigure|ros::|rosrun|roslaunch` в `src/` (кроме
  `autotest`), `launch/*.py`, `config/`, `CMakeLists.txt`, `package.xml`: остались только комментарии
  (`ros::names::validate` в `led.cpp`, `ros::spinOnce()` в шапке `simple_offboard.cpp`, `rospy.ServiceProxy`
  в описании `service_proxy`); скрипт `src/www` удалён.
- `simple_offboard.cpp` переведён на `clover::readMavrosFrames` из `src/mavros_frames.hpp`, своя копия
  удалена. Порядок тот же: параметры `local_frame`/`fcu_frame` → mavros (до 5 с) → `map`/`base_link`.
- `-Wall -Wextra` включены на весь пакет (`add_compile_options`), включая сгенерированный код интерфейсов;
  полная пересборка предупреждений не дала, код править не пришлось.
- **`src/camera_stream`**: обёртка над `mjpg_streamer` (`camera_stream <устройство> <порт>`), ROS не
  использует, на неё никто не ссылается (ни launch, ни CMake), не устанавливается. В ROS 2 её роль
  выполняет `web_video_server`. Удалена.
- **`src/www`**: `ROSWWW_DEFAULT=clover rosrun roswww_static update` — генерация статического сайта
  пакетом `roswww_static`, которого для Jazzy в apt нет. Только ROS 1, не устанавливается, в ROS 2 не
  работает. Удалён. Веб-часть — отдельный этап, там понадобится замена.
- `src/autotest` не трогался (в нём остался `rospy`).
- **Удалены** (с разрешения): `launch/aruco.launch`, `clover.launch`, `led.launch`, `main_camera.launch`,
  `mavros.launch`, `simulator.launch`, `launch/mavros_config.yaml`; `test/basic.test`, `test/offboard.test`;
  `test/smoke_shell.py`, `smoke_camera_markers.py`, `smoke_rc.py`, `smoke_vpe_publisher.py`, `smoke_led.py`,
  `smoke_optical_flow.py` (логика в `test/test_*.py`); `src/camera_stream`, `src/www`. Оригиналы есть в
  истории git и в `CopterExpress/clover`.

### CI

- `.github/workflows/ci.yml`: контейнер `ros:jazzy`, `rosdep install`, `pymavlink` через pip, геоид,
  `colcon build` и `colcon test` для `aruco_pose`, `led_msgs`, `clover` с `-DBUILD_TESTING=ON`.
  **Здесь не запускался и не проверялся** (проверен только синтаксис YAML). В CI `rosdep` поставит
  `web_video_server`, `topic_tools` и остальные пакеты, которых нет в этом окружении, поэтому там впервые
  выполнятся `test_web_video_server` и `rangefinder_relay` — возможны падения в непроверенных местах.

## Веб (этап H)

Проверено: `roswww_static` (6 тестов), установка `www` (`test_www.py`), протокол rosbridge с клиентом на
tornado (`test_web.py`). **Страницы в браузере не открывались**, `viz.js`, `gcs.js`, `topics.js` не
запускались ни в каком JS-движке.

### Порядок запуска

1. `ros2 launch clover clover.launch.py` (аргументы `rosbridge:=true`, `web_video_server:=true` по
   умолчанию): `rosbridge_websocket` + `rosapi` на порту 9090, `tf2_web_republisher`, `web_video_server` на 8080.
2. `ros2 run roswww_static update` — ссылки `~/.ros/www/clover` → `share/clover/www`.
3. Статический сервер (nginx и т. п., с `follow symlinks`) на `~/.ros/www`; страницы ищут rosbridge на
   `ws://<host>:9090`, `web_video_server` на `<host>:8080`.

### Установка `www`

- `www` ставится в `share/clover/www` без `CATKIN_IGNORE` и docs; `clover.log` и `clover_version` — симлинки на
  `/var/log/clover.log` и `/etc/clover_version` (исходные заглушки в репозитории не тронуты).
  `www/CATKIN_IGNORE` остался в репозитории, не устанавливается.
- `roswww_static` — новый пакет в корне репозитория (в Jazzy его нет): `update` ищет `www` в
  `share/<пакет>` через `ament_index_python`.

### Что сверено с установленными пакетами

- Версии: `rosbridge_server` 2.7.1, `tf2_web_republisher` 1.0.0, `web_video_server` 3.1.0.
- **`tf2_web_republisher` 1.0.0: только action** `/tf2_web_republisher`
  (`tf2_web_republisher_interfaces/action/TFSubscription`, цель с `source_frames`, `target_frame`,
  `angular_thres`, `trans_thres`, `rate`; transforms приходят в feedback, результат пустой). Сервиса
  `republish_tfs` (его ждёт `ROSLIB.TFClient` в режиме без groovyCompatibility) нет, хотя
  `tf2_web_republisher_interfaces/srv/RepublishTFs` в пакете интерфейсов объявлен. Исполняемый файл —
  `tf2_web_republisher_node` (в `launch/clover.launch.py` уже так).
- `rosbridge_server` 2.7.1 умеет `send_action_goal` / `cancel_action_goal` и шлёт `action_feedback`;
  запуск — `rosbridge_websocket_launch.xml` (в `clover.launch.py` включается через `AnyLaunchDescriptionSource`),
  вместе с `rosapi`. Типы сообщений в rosbridge — формы ROS 2 (`mavros_msgs/msg/State`), rosapi
  отвечает так же (проверено: `/rosapi/topics`, `/rosapi/topic_type`).
- `web_video_server` 3.1.0: страница `/stream_viewer?topic=` есть (обработчик `handle_stream_viewer`),
  порт по умолчанию 8080, `/stream?topic=`, `/snapshot?topic=`.
- Имена топиков маркеров на запущенных узлах: `/aruco_detect/visualization`, `/aruco_map/visualization`
  (узлы из `aruco.launch.py`, оба `MarkerArray`; первый reliable/volatile, второй transient local),
  `/main_camera/camera_markers` (transient local, `camera_markers` в пространстве имён `main_camera`; сам
  узел публикует после первого `camera_info`). Совпадают с `viz.js` без правок.
- Ссылка на описание типа: `https://docs.ros.org/en/jazzy/p/<пакет>/msg/<Тип>.html` (ответ 200;
  вариант с `/interfaces/msg/` — 404).

### Выбор схемы TF для `viz.js`: вариант А

Вендорный `roslib.js` (ROS 1) не меняется. `ROSLIB.TFClient` с ROS 2 не работает: ждёт сервис
`/republish_tfs` типа `tf2_web_republisher/RepublishTFs`, а в режиме action ходит к
`tf2_web_republisher/TFSubscriptionAction` и не обрабатывает `action_feedback` (адаптер сокета их
отбрасывает).

- **А (выбран):** в `viz.js` класс `TFActionClient` с теми же методами, что использует ROS3D
  (`subscribe`, `unsubscribe`, `fixedFrame`): шлёт `send_action_goal` в rosbridge, читает `action_feedback`
  из того же сокета, при смене набора фреймов отменяет цель и шлёт новую, после переподключения шлёт
  цель заново. Работает на готовом пакете, трафик как в ROS 1 (только изменившиеся фреймы, пороги
  `angular_thres`/`trans_thres`, 10 Гц), tf считает сервер.
- Б (подписка на `/tf` и `/tf_static` в браузере) отвергнут: пришлось бы писать в JS буфер tf с поиском по
  дереву и интерполяцией, получать весь `/tf`, а `/tf_static` приходит один
  раз, и поздний подписчик зависит от QoS rosbridge. Больше кода и больше мест для ошибок.
- Цель action не завершается сама (результат пустой), завершается отменой. Что происходит с целью при
  закрытии вкладки (отмена со стороны rosbridge или нет), не проверялось.
- Фиксированный фрейм сцены — тот же, что в ROS 1 (`fixedFrame` передаётся в `setScene`).

### Правки страниц

- `topics.js`: сравнение типа `sensor_msgs/msg/Image`, тип разбирается как `pkg/msg/Type`,
  `/rosdistro` заменён на константу `jazzy` (параметра в ROS 2 нет), время `sec`/`nanosec`.
- `gcs.js`: типы `mavros_msgs/msg/State`, `mavros_msgs/msg/StatusText`, `sensor_msgs/msg/BatteryState`.
- `viz.js`: текст ошибки `clover.launch.py`.
- Вендорные `roslib.js`, `ros3d.js`, `three.min.js`, `eventemitter2.js`, `yaml.js` не менялись.

### Не работает или не проверено

- **`/vehicle_marker` (модель дрона в `viz.html`) не появится.** В ROS 1 его публиковал узел `visualization` из
  `mavros_extras`; в Jazzy его нет (проверено по `mavros_plugins.xml` и `lib/mavros_extras`). Вариант замены, не
  реализован: маленький узел на C++/Python, который подписывается на `mavros/local_position/pose` и публикует
  `MarkerArray` (`MESH_RESOURCE` или набор `CUBE`/`CYLINDER`) в `/vehicle_marker` с `frame_id` фрейма дрона.
  Меш пришлось бы раздавать с веб-сервера, а не через `package://`.
- В `index.html` ссылки на `clover_blocks` (пакет не портирован) и Butterfly (веб-терминал, не ставится) оставлены,
  **обе нерабочие**.
- **Не проверялось в браузере:** любая страница, ROS3D-сцена, голосовые сообщения `gcs.html`, `console.html`,
  `aruco_map.html`, отображение маркеров, ссылки на `web_video_server`, работа за nginx.
- Проверено только протоколом (`test_web.py`): получение сообщений, `rosapi/topics` и `topic_type`, подписка без
  типа, поздний подписчик на transient local (`mavros/state`, `camera_markers`, `aruco_map/visualization`),
  action TF (одна цель, отмена, новая цель с двумя фреймами). Подписчик `aruco_detect/visualization` не
  проверялся (publisher volatile, как в узле); `camera_markers` и `aruco_*` в тесте заменены mock-публикаторами с
  теми же именами и QoS. `web_video_server` в тесте не запускается (его запуск проверяет `basic.py`).
- **На железе и в симуляции ничего не проверялось.**

### Найдено в оригинале, не исправлялось

- `gcs.js`: `message.cell_voltage[0].toFixed(2)` падает, если массив `cell_voltage` пустой. Заполняет ли mavros
  для ROS 2 `cell_voltage` всегда, не проверялось.
- `viz.js`: при потере соединения сцена не восстанавливается (`ros.on('error')` только показывает alert).

## Чего нет без драйверов

- **Лента `ws281x`.** Драйвера для ROS 2 нет. Узел `led` без сервиса `led/set_leds` и топика
  `led/state` ждёт их бесконечно и сервис `led/set_effect` не создаёт.
- **Дальномер `vl53l1x`.** Драйвера для ROS 2 нет, топик `rangefinder/range` никто не публикует:
  `selfcheck` всегда сообщает `Rangefinder: no rangefinder data from Raspberry`, а `Vision position estimate`
  не может определить, что дрон стоит на полу.

## Образ для Raspberry Pi 4 и Pi 5 (этап I)

Скрипты в `image/`, описание в `image/README.md`. **Образ ни разу не собирался, не запускался на Pi 4 и Pi 5, workflow
`.github/workflows/build-image.yaml` не запускался.** Проверено статически: `bash -n`, shellcheck (0 замечаний), yamllint, actionlint,
`systemd-analyze verify` (только ожидаемые замечания о правах файлов на `/mnt/c` и ещё не установленном скрипте),
`image/test/test_guard.sh` (скрипты отказываются работать вне сборки) и `image/test/test_functions.sh` (чистые функции на временных
файлах, включая таблицу разделов обычного файла через `sfdisk`). Монтирование, loop-устройства, chroot, `resize2fs` не запускались
(нет root, и защита от этого стоит намеренно).

### Базовый образ

- `ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz` с `cdimage.ubuntu.com/releases/24.04/release/` (curl: 200, 1 368 926 404 байт),
  SHA256 `b23371a5...4351e` взят из `SHA256SUMS` того же каталога и закреплён в `image/image-build.sh`. Подпись `SHA256SUMS.gpg` не проверяется.
- Пользователь `pi`/`raspberry` создаётся при сборке (в оригинале он уже был в Raspberry Pi OS), а не cloud-init, потому что в chroot
  нужны его домашний каталог, `rosdep update`, `colcon build`. В `user-data` он перечислен, `ubuntu` не создаётся. **Поведение cloud-init при уже
  существующем пользователе не проверялось** (знание, не прогон).

### Расхождения с оригиналом

- **Нет:** `roscore.service`, `ROS_HOSTNAME`/`ROS_IP`, monkey → nginx (`/home/pi/.ros/www`, порт 80; нужен `o+x` на `/home/pi`, выставляется),
  Butterfly (ссылка в `www/index.html` нерабочая, как записано на этапе H), pigpio/`python3-pigpio`/`rpi_ws281x` (не работают на Pi 5, образ один для обеих
  моделей), gitbook и документация, Node.js 10, ptvsd (его заменил `debugpy`, никто не использует), pyzbar и `libzbar0` (нужен только `test_qr.py`
  оригинала), пакет `clever`, `mjpg-streamer` (в noble нет, роль у `web_video_server`), `ntpdate` (есть `systemd-timesyncd`), `rsyslog.conf` и
  `rsysrot.sh` (остался только `SystemMaxUse=200M` у journald), `/etc/sudoers.d/ros_python_paths` (переменные ROS 1; нужен ли `env_keep` для
  `AMENT_PREFIX_PATH` и `PYTHONPATH` при `sudo`, не выяснялось). `VL53L1X` из `clover/requirements.txt` в образ не ставится (этап J).
- **`rc.local` → `clover-firstboot.service`** (oneshot, флаг `/var/lib/clover/firstboot.done`), перезагрузка после него не делается
  (в оригинале была). Генерация `clover-XXXX` та же по смыслу: 4 цифры.
- **Монтирование:** `/run` в chroot отдельный tmpfs (host systemd не должен быть доступен), `/dev` и `/sys` через `--rbind` и `--make-rslave`, весь
  `image-chroot.sh` работает в приватном пространстве имён монтирования (`unshare --mount --propagation private`). Токен `/etc/clover_image_build`
  и переменная `CLOVER_CHROOT_TOKEN` не дают запустить скрипты изнутри образа на хосте. Это написано по знанию, не по прогону.
- **Сеть:** NetworkManager вместо dhcpcd/wpa_supplicant/dnsmasq. Профиль `clover-ap` (keyfile): `proto=rsn` (в оригинале WPA и RSN), `band=bg`,
  канал не задан. **DHCP-пул NetworkManager (режим `shared`) свой, `192.168.11.10..254`, а не `100..200` как в оригинале**: настраивается плохо.
  Имена `clover`, `coex` заданы через `/etc/NetworkManager/dnsmasq-shared.d/`. Профиль клиентского режима заранее не создаётся (SSID и пароль неизвестны),
  в README команды `nmcli`. Страна Wi-Fi: `cfg80211.ieee80211_regdom=GB` в `cmdline.txt` вместо `country=GB` в `wpa_supplicant.conf`.
  `NetworkManager-wait-online` и `systemd-networkd-wait-online` отключены (иначе растёт время загрузки). `clover.local` не появится, как и в оригинале
  (только `clover-XXXX.local`).
- `clover.service`: `Restart=on-failure`, `RestartSec=3` (в оригинале перезапуска не было), `fcu_conn` не задан (по умолчанию в нашем launch `usb`, как
  в оригинале). Окружение берётся из `/etc/clover/ros-env.sh`, тот же файл читает `~/.bashrc`.
- **RMW:** по умолчанию остаётся `rmw_fastrtps_cpp`, потому что все тесты пакета прогонялись с ним. `rmw_cyclonedds_cpp` ставится, но с `clover` не проверялся.
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `ROS_DOMAIN_ID` не задаётся.
- **pymavlink:** `pip install --break-system-packages pymavlink==2.4.49` в `/usr/local` (apt-пакета нет; ROS-модули живут в системном интерпретаторе,
  поэтому не venv). Версия та, с которой прогонялись тесты на этапе E.
- **colcon:** `--parallel-workers 1`, `MAKEFLAGS=-j$(nproc/2)`; `--executor sequential` не используется (при одном worker он ничего не меняет).
- **Тесты в образе:** сборка идёт с `-DBUILD_TESTING=OFF`, поэтому `image-validate.sh` отдельно собирает и тестирует только `roswww_static` во временном
  каталоге. Полный `colcon test --packages-select clover` в chroot не запускается (нужны mavros_node, DDS, мультикаст), он остаётся в `ci.yml`.
- `systemd-analyze verify` в `image-validate.sh` только предупреждает (в chroot без systemd как PID 1 поведение не проверялось), остальные проверки ломают сборку.
- `libcamera`: `ros-jazzy-camera-ros` и `ros-jazzy-libcamera` ставятся, но **собраны ли в них pipelines Raspberry Pi (`rpi/vc4` для Pi 4, `rpi/pisp` для Pi 5), не
  известно**; `image-validate.sh` ищет строки `vc4` и `pisp` в библиотеках и только предупреждает. CSI-камера не проверялась, по умолчанию в launch остаётся USB.
- `vcgencmd`: `libraspberrypi-bin` в noble нет (`apt-cache policy` пуст). `selfcheck` на образе печатает info `could not call vcgencmd binary; not a Raspberry
  Pi?` и проверку питания не делает. Замена (sysfs `hwmon`, `in0_lcrit_alarm`) из памяти, не проверялась и не написана.
- Не отключались без решения владельца: `snapd` и прочее лишнее в server-образе (влияет на время загрузки, которое замеряет `selfcheck`).
- Лимиты раннера (диск ~14 ГБ, время сборки 1,5 ч) не проверялись, это оценки.

### UART к полётному контроллеру: источники

Итог для `fcu_conn:=uart` (`/dev/ttyAMA0:921600` в `launch/mavros.launch.py`): **по источникам расхождения нет, на железе не проверено.**
Для Pi 4 нужен `dtoverlay=disable-bt`, для Pi 5 нужен `dtoverlay=uart0-pi5`; без них `/dev/ttyAMA0` либо занят Bluetooth (Pi 4), либо его на GPIO14/15 нет (Pi 5).
`/dev/serial0` для FCU не годится: на Pi 5 он указывает на `/dev/ttyAMA10` (отладочный разъём). Launch не менялся. Если на железе имя окажется другим,
предложение: udev-симлинк `/dev/clover-fcu` или параметр launch (с вопросом к владельцу).

Что прочитано (все 2026-10-07):

1. `raspberrypi/linux`, ветка `rpi-6.12.y`, `arch/arm/boot/dts/overlays/README`: `uart0-pi5`: «Enable uart 0 on GPIOs 14-15. Pi 5 only.»; файл
   `uart0-pi5-overlay.dts` (`compatible = "brcm,bcm2712"`, цель `&uart0`, `uart0_pins`).
2. `arch/arm64/boot/dts/broadcom/bcm2712-rpi.dtsi`: `serial0 = &uart0; ... serial10 = &uart10;`, `stdout-path = "serial10:115200n8"`;
   `bcm2712-rpi-5-b.dts`: `uart10` это отладочный 3-pin разъём. **Что `ttyAMA<N>` берёт номер из алиаса `serialN`, это моё знание драйвера PL011, в источниках
   не написано.**
3. Документация Raspberry Pi (`raspberrypi/documentation`, PR #3294, вторичный источник): `/dev/serial0` на Pi 5 указывает на `/dev/ttyAMA10`. Параметр
   `enable_rp1_uart=1` настраивает вывод **прошивки** на GPIO14/15, не порт ядра, не используется. Поиск по сети по этому вопросу дал противоречивые
   выдержки (одна говорила, что GPIO14/15 это `ttyAMA10`), поэтому опираюсь на файлы DTS, а не на них.
4. Pi 4: `bcm2711-rpi-4-b.dts`: `stdout-path = "serial1"` (mini UART), `&uart0` связан с BT-модулем; `disable-bt-overlay.dts`: включает UART0 на GPIO14/15,
   отключает BT, ставит `serial0` на PL011 (`/soc/serial@7e201000`), `serial1` на mini UART.
5. Ubuntu: `ubuntu.com/hardware/docs/boards/how-to/special_hardware/rpi-config-txt/` (`config.txt` в `/boot/firmware`, `enable_uart=1` в server-образе);
   packages.ubuntu.com, noble arm64: `uart0-pi5.dtbo` и `disable-bt.dtbo` в `linux-modules-6.8.0-1004-raspi`.

**Оговорка:** п.1, 2, 4 это дерево ядра Raspberry Pi, а Ubuntu собирает свой `linux-raspi`; алиасы в DTB Ubuntu могли отличаться. Поэтому
`image-validate.sh` печатает алиасы `serial*` из `bcm2711-rpi-4-b.dtb` и `bcm2712-rpi-5-b.dtb` образа (`dtc -I dtb`) и проверяет наличие обоих `.dtbo`.
Пример вывода появится только после первой сборки.

### Pi 4 и Pi 5

Один образ, различия в `config.txt` (блок `# clover begin` ... `# clover end` в конце файла, секции `[pi4]` и `[pi5]`) и в `/etc/clover_hw`
(`/proc/device-tree/model` при первой загрузке). Pi 4 здесь проверить нельзя, Pi 5 тоже.

| Место | Общее | Pi 4 | Pi 5 |
|---|---|---|---|
| UART к FCU | `/dev/ttyAMA0`, из `cmdline.txt` убраны `console=serial0|ttyAMA0|ttyAMA10|ttyS0,...`, `serial-getty@ttyAMA0` замаскирован | `dtoverlay=disable-bt` | `dtoverlay=uart0-pi5` |
| I2C, SPI | `dtparam=i2c_arm=on`, `dtparam=spi=on`, модули `i2c-dev` и `spidev`, группы `i2c`, `spi`, `gpio` | на SoC | на RP1 |
| Питание | | `vcgencmd` нет (см. выше) | USB ограничен 600 мА без БП 5 А; `usb_max_current_enable=1` оставлен **закомментированным** (на БЭК без PD просадка опаснее) |
| Лента, дальномер | драйверов нет (этап J) | pigpio бы работал, но не ставится | pigpio и `rpi_ws281x` не работают (RP1) |
| CSI | `camera-ros` ставится, по умолчанию USB (`v4l2_camera`) | pipeline `rpi/vc4`, шлейф 15 пин | pipeline `rpi/pisp`, шлейф 22 пин |

### Найдено в оригинале `builder/`, не переносилось и не исправлялось

Читалось, не запускалось.

- `image-init.sh` вставляет команду в `/etc/rc.local` по номеру строки (`sed -i "19a..."`): ломается, если файл Raspberry Pi OS изменится.
- `image-build.sh`, `get_image`: строка `echo_stamp "Downloading complete" "SUCCESS" \` склеивается со следующей `else echo_stamp ...; fi`, `else` становится аргументом
  команды, ветки `else` нет. Проверено запуском фрагмента: печатается `Downloading complete SUCCESS else echo_stamp ...`, сообщение «already downloaded» не выводится никогда.
- `hardware_setup.sh`: раздел `/boot` и `root=/dev/mmcblk0p2` прописываются жёстко (не подходит для USB/NVMe-загрузки).
- `image-software.sh`: ключи apt через `apt-key adv --keyserver` (устарело), в `image-ros.sh` `export ROS_IP='127.0.0.1'` «для тестов».
- `echo_stamp` во всех скриптах оригинала выводит текст через `echo -e ${TEXT}` без кавычек.

### Этап J (не делался)

- **Драйвер ленты на Pi 4 и Pi 5:** интерфейс ядра `spidev` (одинаков для обеих моделей; не pigpio и не `rpi_ws281x`). Для Pi 4 при SPI нужна
  правильная частота ядра/SPI, **проверить при реализации**. Контракт: сервис `led/set_leds` (`led_msgs/srv/SetLEDs`), топик `led/state`
  (`led_msgs/msg/LEDStateArray`), QoS **reliable + transient local, depth 1** (иначе узел `led` не стартует, см. раздел LED), параметры как у `ws281x` в
  `config/led_notify.yaml` и `launch/led.launch.py`.
- **`vl53l1x` по I2C** (`/dev/i2c-1`, адрес 0x29, `smbus2`): топик `rangefinder/range` (`sensor_msgs/msg/Range`), QoS best effort, volatile, depth 1
  (совместимо с `simple_offboard`, `selfcheck`, примерами). `frame_id` и границы дальности как в оригинале `vl53l1x`.

### Что не проверено совсем

Сборка образа целиком, запуск CI, загрузка на Pi 4 и Pi 5, расширение корня, `clover-firstboot`, точка доступа и DHCP, имена `clover`/`coex`, avahi, nginx
и права, UART/I2C/SPI, имя `/dev/ttyAMA0`, CSI-камера и libcamera, `clover.service` на устройстве, mavros 2.16.0 (в том числе `set_attitude`),
ветка сборки mavros из исходников, `ros2-apt-source` 1.3.0 в chroot, `systemd-analyze verify` внутри chroot, `rosdep install` на arm64.

## Нестабильные тесты (CI)

Состояние: правки тестов проверены только на WSL в эмуляции CI (`taskset -c 0-1`, `/dev/shm` 64 МБ через `unshare -Urm`, без CPU-жгутов),
по 5 прогонов. **Реальный раннер GitHub не проверялся**, `ci-experiment.yml` не запускался.

### Что падало в CI и что с этим сделано

| Тест | Симптом в CI | Воспроизведено на WSL? | Правка |
|---|---|---|---|
| `aruco_pose_basic` (`test_debug`, `test_map`, `test_map_debug`, `test_markers`, `test_visualization`: меняются от прогона к прогону) | `no message on aruco_detect/debug`, `aruco_map/pose`, `aruco_map/debug` | да, 3 из 5 прогонов упали | подписки на выходы создаются до тестов (`PRELOAD`), первое сообщение запоминается, ожидание готовности конвейера `READY` до 120 с, в сообщении об ошибке число полученных сообщений |
| `test_web::test_latched_markers` | `no expected message in 10 s` | да, 1 из 5 | подписка повторяется (`latched_message`, период 5 с, общий срок 60 с) |
| `test_web::test_rosapi` | `Service /rosapi/topics does not exist` | нет (в логе CI, мной не виден) | ожидание сервисов `/rosapi/topics`, `/rosapi/topic_type` до 30 с через граф, ожидание появления тестовых топиков в ответе `rosapi` до 30 с; `test_tf` ждёт action `/tf2_web_republisher` |
| `test_optical_flow::test_standalone` (`integration_time_us`) | assert на `integration_time_us` | **нет**, старая версия проходит 5 из 5 | см. ниже |
| ещё 2 падения последнего прогона CI | не известны мне | нет | жду лог |

Причина `aruco_pose_basic`: `aruco_detect` подписан на `image_raw` с `rmw_qos_profile_sensor_data` (best effort), картинка 640x480 `rgb8` (921 600 байт, 10 Гц,
`image_publisher_node`). Предупреждение `Image messages received: 0 / CameraInfo messages received: 10` печатает не наш код, а `image_transport::CameraSubscriber`
(строка в `libimage_transport.so`): картинки до подписчика не доходят, а `CameraInfo` (маленький) доходит. Это воспроизводится и на WSL (в логе той же эмуляции
0–2 картинки за 10 с).

### Замеры доставки (WSL, `image_publisher_node` + best effort подписчик, как `aruco_detect`, + reliable подписчик, 20 с, 3 повтора)

Доля картинок, дошедших до best effort подписчика (по отношению к reliable подписчику), в процентах по прогонам:

| Вариант | Доставлено, % | Максимальная пауза, с |
|---|---|---|
| Fast DDS по умолчанию, `/dev/shm` 1 ГБ, 2 ядра | 0, 0, 13 | 0, 0, 3,5 |
| Fast DDS по умолчанию, `/dev/shm` 64 МБ, 2 ядра | 0, 15, 18 | 0, 3,0, 1,7 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`, 2 ядра | 46, 11, 42 | 1,4, 2,1, 0,8 |

Что это значит и чего это не значит:

- Большие картинки по best effort на двух ядрах теряются **сильно во всех трёх вариантах**; SHM-транспорт с большим `/dev/shm` не лучше, чем с 64 МБ, поэтому
  гипотеза «виноват только размер `/dev/shm` контейнера» этими замерами не подтверждена. UDPv4 в среднем лучше, но выборка из 3 прогонов с большим разбросом
  не позволяет назвать эффект доказанным.
- **Первый вариант матрицы (11 вариантов по 2 прогона) выброшен**: скрипт замера не завершал `image_publisher_node` (убивался `ros2 run`, но не его дочерний
  процесс), так что к концу работало больше десятка издателей, и цифры (в том числе «ASYNCHRONOUS лучше», «UDP 91–104 %») ненадёжны. Файл остался в
  scratchpad, выводов из него не делаю. По этой причине **`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`, увеличенные буферы сокетов, уменьшенное изображение,
  CPU-жгуты на WSL не измерены**. `rmw_cyclonedds_cpp` не установлен (`ros-jazzy-rmw-cyclonedds-cpp`), не проверялся.
- Матрицу по окружению на раннере делает `.github/workflows/ci-experiment.yml` (по умолчанию / `--shm-size=1g` / `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` /
  `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`, по 5 прогонов `colcon test` для `aruco_pose` и `clover`, `fail-fast: false`). В `ci.yml` переносить только то,
  что даст эффект; потом удалить `ci-experiment.yml`. Пока `ci.yml` не менялся по окружению.

### Результаты прогонов (эмуляция CI на WSL, 2 ядра, `/dev/shm` 64 МБ, 5 прогонов каждого)

| Тест | До правок (HEAD) | После правок |
|---|---|---|
| `clover/test/test_optical_flow.py` | 5 из 5 | 5 из 5 |
| `clover/test/test_web.py` | 2 из 5 | 5 из 5 |
| `aruco_pose/test/basic.py` | 2 из 5 (время 49–142 с) | 5 из 5 (время 23–124 с) |

Мало прогонов для вывода о доле падений; 20 прогонов не делал (WSL 3 ГБ, ограничение по процессам). Один прогон `basic.py` после правки занял 124 с, значит
картинки теряются и теперь, тест просто дожидается.

### `test_optical_flow`

На WSL не падал ни до, ни после правки, поэтому причина по логу CI **не подтверждена**. Что видно по коду: `integration_time_us` равен разности `header.stamp`
двух кадров (`optical_flow.cpp:246`), а тест проверял его границами `(0.5..2)/RATE` по номинальной частоте и смещение только для одного кадра; таймер издателя в
тесте на загруженной машине запаздывает, кадр может быть потерян. Правка (допуски по значениям `0,2` пикселя и `1e-3` **не расширялись**):

- `MockCamera` запоминает метки времени и номера кадров. Для каждого `flow` проверяется, что `stamp − integration_time_us` совпадает (±2 мкс, усечение до мкс) с меткой реально
  отправленного кадра.
- Проверки значений (смещение, `integrated_x/y`) выполняются на последнем `flow`, собранном из двух **соседних** кадров (ожидание до 30 с); `shift` и `velocity` берутся с той же
  меткой времени.
- Граница периода `(0.5..2)/RATE` применяется к **медиане** `integration_time_us` соседних кадров, а не к последнему сообщению.
- `time.sleep` заменены ожиданием номера кадра: после `enabled=False` ни одного `flow` от кадров, отправленных позже; после включения больше 5 новых `flow`. Проверка
  `disable_on_vpe` сравнивает метки времени `flow` с метками отправленных поз (правило узла: поза моложе 0,1 с), без паузы по стене.

### Расширенные сроки (только верхние границы ожидания события, возвращаются сразу при получении)

`aruco_pose` `READY_TIMEOUT` 120 с (новая константа), ожидания в `optical_flow` 5 → 30 с, `test_latched_markers` 10 → 60 с (с повторной подпиской), `rosapi` 30 с.
Обоснование: события приходят сразу, срок влияет только на время до отказа; значения допусков не менялись.

### Временное

- `ci.yml`: `colcon test ... --retest-until-pass 3` (TEMPORARY) и шаг, который печатает строки с `retest`/`re-run` из вывода. **Формат этих строк не проверялся**, при
  отсутствии совпадений печатается «no retries found». Снять, когда тесты проходят без повторов.
- Список нестабильных тестов (из логов CI и эмуляции): `aruco_pose_basic` (`test_debug`, `test_map`, `test_map_debug`, `test_markers`, `test_visualization`),
  `test_web::test_latched_markers`, `test_web::test_rosapi`, `test_optical_flow::test_standalone`. Ещё 2 падения будут добавлены по логу.

### RMW и режим публикации: предложение для образа (не применено)

`image/assets/ros-env.sh` **не менялся**. Данных для решения пока мало: на раннере и на Pi 4/5 ничего не измерено, на WSL лучшим выглядит `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
но с большим разбросом. Предложение к обсуждению после `ci-experiment.yml` и первых запусков на Pi: если на раннере UDPv4 или `ASYNCHRONOUS` дают эффект, добавить это же
в `ros-env.sh` (там нет ограничения `/dev/shm` в 64 МБ и больше ядер, так что эффект на образе может быть другим); камера (`v4l2_camera`) и `aruco_pose` на устройстве
работают в одном контейнере компонентов (`main_camera_container`), то есть через DDS идут только картинки к внешним подписчикам (`web_video_server`, `rviz`). По умолчанию
оставить `rmw_fastrtps_cpp` без дополнительных переменных до измерений на железе.
