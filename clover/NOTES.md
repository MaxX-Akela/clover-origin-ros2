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

## Чего нет без драйверов

Будет дополняться на этапах C–F (лента `ws281x`, дальномер `vl53l1x`, `cv_camera`, веб-пакеты).
