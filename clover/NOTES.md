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
  не получает.
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
- Не проверено: `mavros/distance_sensor/*` (топики создаются по конфигурации плагина).

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

Проверено сборкой с `-Wall -Wextra` и дымовыми прогонами с mock-издателями (`test/smoke_<узел>.py`),
плюс совпадение издателей и подписчиков с запущенным `mavros_node` без автопилота. С камерой, реальными
кадрами и автопилотом ничего не проверялось.

### Общее

- **Кадры.** `vpe_publisher` и `optical_flow` получают кадры через `src/mavros_frames.hpp` по той же
  схеме, что `simple_offboard`: параметры `local_frame`/`fcu_frame` (**новые**, по умолчанию пустые) →
  узел `mavros/local_position` (`tf.frame_id`, `tf.child_frame_id`, ожидание до 5 с) → `map`/`base_link`
  с предупреждением. `simple_offboard.cpp` на общий заголовок не переведён (в этап не входил), логика
  в нём продублирована.
- **QoS.** Подписки на чужие данные (`mavros/*`, `camera_info`, `image_raw`, `~/pose`, `~/pose_cov`) —
  best effort, volatile, глубина 1. Публикации в mavros — reliable, глубина 1. Latched-топики
  (`camera_markers`, `state_latched`) — reliable + transient local.
- Приватные имена (`~pose`, `~vpe`, `~reset`, `~shift`, `~debug`, ...) остались приватными (`~/...`).
- Флаги `-Wall -Wextra` заданы только для целей этапа C, а не для всего пакета.

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

Проверено сборкой с `-Wall -Wextra` и дымовым прогоном `test/smoke_led.py` с mock-драйвером
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

## Чего нет без драйверов

- **Лента `ws281x`.** Драйвера для ROS 2 нет. Узел `led` без сервиса `led/set_leds` и топика
  `led/state` ждёт их бесконечно и сервис `led/set_effect` не создаёт.
- **Дальномер `vl53l1x`.** Драйвера для ROS 2 нет, топик `rangefinder/range` никто не публикует:
  `selfcheck` всегда сообщает `Rangefinder: no rangefinder data from Raspberry`, а `Vision position estimate`
  не может определить, что дрон стоит на полу.

Будет дополняться на этапе F (`cv_camera`, веб-пакеты).
