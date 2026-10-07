# Образ Clover для Raspberry Pi 4 и Raspberry Pi 5

Скрипты сборки образа Ubuntu 24.04 (arm64) с ROS 2 Jazzy и пакетами этого репозитория (`aruco_pose`, `led_msgs`,
`roswww_static`, `clover`). Это порт каталога `builder/` оригинала ([CopterExpress/clover](https://github.com/CopterExpress/clover),
Debian Buster, ROS Noetic, Docker).

**Статус: образ ни разу не собирался и не запускался ни на Pi 4, ни на Pi 5.** Проверялись только `bash -n`, shellcheck,
yamllint, actionlint, `systemd-analyze verify` и пробный прогон чистых функций на временных файлах (`image/test/`).
Что работает на железе (загрузка, Wi-Fi, UART, камера, mavros с автопилотом), неизвестно. Поддерживаются Pi 4 и Pi 5,
Pi 3 и Zero нет; Pi 400 и CM4 не заявляются.

## Как собирать

Только на нативном arm64 в CI (`.github/workflows/build-image.yaml`, раннер `ubuntu-24.04-arm`; workflow ни разу не запускался).
Docker и qemu не используются. Скрипты, которые подключают loop-устройства и монтируют образ, отказываются запускаться
вне CI (код выхода 64), если нет `CI=true` или флага `--i-know`; под WSL нужен именно `--i-know`. Скрипты, которые
выполняются внутри образа, не запускаются вообще нигде, кроме `scripts/image-chroot.sh exec` (токен в `/etc/clover_image_build`).

```
sudo -E image/image-build.sh            # в CI (CI=true), нативный arm64
```

Переменные окружения (все необязательны): `CLOVER_IMAGE_VERSION`, `SOURCE_IMAGE_URL`, `SOURCE_IMAGE_SHA256`, `IMAGE_SIZE` (8G),
`IMAGES_DIR`, `CACHE_DIR` (по умолчанию `image/out/`), `CLOVER_MAVROS_SOURCE` (`auto`, `apt`, `source`), `CLOVER_BUILD_JOBS`.

Порядок: скачивание базового образа Ubuntu 24.04.5 preinstalled server arm64+raspi с проверкой SHA256 → расширение до `IMAGE_SIZE` →
`image-init.sh` → копирование репозитория → `image-software.sh` → `image-ros.sh` → `image-network.sh` → `image-hardware.sh` →
`image-validate.sh` → `image-cleanup.sh` → усечение → `xz`.

| Файл | Назначение |
|---|---|
| `scripts/image-resize.sh`, `scripts/image-chroot.sh` | замена одноимённых скриптов контейнера оригинала: рост/усечение образа, монтирование и chroot (с `trap` и размонтированием) |
| `scripts/check-apt-packages.sh` | быстрая проверка arm64-пакетов и версии mavros (первый шаг CI) |
| `scripts/install-ros-apt-source.sh` | репозиторий ROS 2 (`ros2-apt-source`) |
| `image-init.sh` | версия, пользователь `pi`, seed cloud-init, journald |
| `image-software.sh` | apt-пакеты, `pymavlink` |
| `image-ros.sh` | ROS 2 Jazzy, rosdep, mavros, геоид, `colcon build`, nginx, `clover.service` |
| `image-network.sh` | NetworkManager, точка доступа, avahi, first-boot |
| `image-hardware.sh` | `/boot/firmware/config.txt`, `cmdline.txt`, UART/I2C/SPI |
| `image-validate.sh`, `image-cleanup.sh` | проверки и зачистка внутри образа |
| `assets/` | unit-файлы, профиль NetworkManager, nginx, cloud-init, udev |
| `test/test_guard.sh`, `test/test_functions.sh` | пробные прогоны без root, без loop и chroot |

## Что получается в образе

- Пользователь `pi`, пароль `raspberry` (как в документации Clover; смените). SSH включён. Пользователь создаётся при сборке,
  cloud-init (`/boot/firmware/user-data`) не создаёт `ubuntu` и не просит менять пароль. Как cloud-init ведёт себя с уже
  существующим пользователем, не проверялось.
- Hostname и SSID `clover-XXXX` (4 цифры) задаёт `clover-firstboot.service` при первой загрузке, туда же пишется `/etc/clover_hw`
  (`pi4`, `pi5`, `unknown`).
- Wi-Fi: NetworkManager, точка доступа `clover-XXXX`, пароль `cloverwifi`, `192.168.11.1/24`, DHCP от NetworkManager,
  имена `clover` и `coex` в DNS точки доступа, avahi (`clover-XXXX.local`). Клиентский режим (точку доступа придётся выключить,
  `wlan0` один):

  ```
  nmcli connection down clover-ap
  nmcli device wifi connect "<ssid>" password "<пароль>"
  # обратно: nmcli connection up clover-ap
  ```

- `clover.service` (то же имя, что в оригинале): `ros2 launch clover clover.launch.py` без аргументов, то есть `fcu_conn:=usb`.
  Для UART допишите `fcu_conn:=uart` в `ExecStart` файла `/etc/systemd/system/clover.service` и выполните `sudo systemctl daemon-reload`.
- Веб: nginx на порту 80 отдаёт `~/.ros/www`; rosbridge (9090) и web_video_server (8080) запускает `clover.launch.py`.
- ROS 2: `/etc/clover/ros-env.sh` (его читают `~/.bashrc` и `clover.service`), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`,
  RMW по умолчанию `rmw_fastrtps_cpp`. Переключение на Cyclone DDS (установлен, **с нашими пакетами не проверялся**):
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- mavros: из apt, если там версия не ниже 2.16.0, иначе из исходников в `/opt/mavros_ws` (эта ветка ни разу не выполнялась).
  Геоид в `/usr/share/GeographicLib`.

## Отличия от образа оригинала

Нет: `roscore.service`, `ROS_HOSTNAME`, monkey (вместо него nginx), Butterfly, pigpio и `rpi_ws281x`, gitbook и документации,
ptvsd, pyzbar, пакета `clever`, `mjpg-streamer`, Node.js. Подробности и причины в `clover/NOTES.md`, раздел «Образ (этап I)».

## Pi 4 и Pi 5

Один образ, различия в секциях `[pi4]` и `[pi5]` файла `/boot/firmware/config.txt`. UART к полётному контроллеру на GPIO14/15:
`dtoverlay=disable-bt` (Pi 4) и `dtoverlay=uart0-pi5` (Pi 5), устройство `/dev/ttyAMA0` (как в `mavros.launch.py` для `fcu_conn:=uart`).
Источники и оговорки: `clover/NOTES.md`. **Имя порта на железе не проверялось.** Ленты нет (драйвера нет), дальномера нет.

## Что можно проверить только на железе

Загрузка и расширение корня на Pi 4 и Pi 5; `clover-firstboot`; точка доступа, DHCP, имена `clover`/`coex`, avahi; nginx и права на
`/home/pi`; `/dev/ttyAMA0` и отсутствие консоли на нём; I2C, SPI; питание и просадки; CSI-камера и libcamera; запуск
`clover.service`; mavros с автопилотом (в том числе `set_attitude` на mavros 2.16.0); время загрузки, на которое смотрит `selfcheck`.
Внутри chroot нет systemd как PID 1 и нет железа, поэтому `image-validate.sh` проверяет файлы, пакеты, окружение ROS и безжелезные тесты.
