# a733_csi_cam_ros2

ROS 2 Humble драйвер для CSI-камеры IMX219 на Orange Pi Zero 3W с SoC Allwinner A733.

Пакет открывает камеру напрямую через V4L2 `VIDEO_CAPTURE_MPLANE`, забирает кадры из `mmap`-буферов, при необходимости уменьшает разрешение через OpenCV и публикует поток в ROS 2:

- `sensor_msgs/msg/Image` в кодировке `bgr8`
- `sensor_msgs/msg/CameraInfo`

Пакет сделан для плат, где IMX219 работает через vendor/BSP-драйвер `sunxi-vin` и модуль ядра `vin_v4l2`. Если на системе доступны библиотека `libAWIspApi.so` и заголовок `AWIspApi.h`, нода включает Allwinner ISP 3A: автоэкспозицию, авто-баланс белого и связанные алгоритмы ISP. Без `AWIspApi` нода все равно собирается и запускается, но изображение может быть темным или нестабильным по экспозиции.

## Где использовать

Целевая связка:

| Компонент | Значение |
|---|---|
| Плата | Orange Pi Zero 3W |
| SoC | Allwinner A733 |
| Камера | Raspberry Pi Camera Module 2 / IMX219, MIPI CSI |
| ОС | Debian/Ubuntu-образ для Orange Pi с ядром 6.6+ и BSP-драйверами камеры |
| ROS | ROS 2 Humble |
| Архитектура | `aarch64` |

Пакет не является универсальным драйвером для всех CSI-камер. Он рассчитан на Allwinner A733 и V4L2-устройство, которое умеет отдавать BGR24 через `VIDIOC_S_FMT`.

## Возможности

- прямой захват кадров из `/dev/video*` без GStreamer и без промежуточного SHM;
- публикация `Image` и `CameraInfo` с единым timestamp;
- настраиваемый image topic, `frame_id`, FPS и входное разрешение;
- опциональный resize на выходе, например захват `1280x960`, публикация `320x240`;
- QoS `RELIABLE` или `BEST_EFFORT`;
- загрузка YAML-калибровки через `camera_info_manager`;
- автоматический retry, если камера временно недоступна;
- попытка включить realtime-приоритет для capture-потока;
- опциональный Allwinner ISP 3A через `AWIspApi`.

## Структура репозитория

```text
a733_csi_cam_ros2/
├── .github/
│   └── workflows/
│       └── ci.yml
├── .gitignore
├── ABOUT.md
├── CHANGELOG.md
├── CMakeLists.txt
├── CONTRIBUTING.md
├── LICENSE
├── package.xml
├── README.md
├── config/
│   └── imx219_1280x960.yaml
├── launch/
│   └── camera.launch.py
└── src/
    └── imx219_camera_node.cpp
```

## ROS-топики

Нода публикует image topic из параметра `topic`. Topic для `CameraInfo` вычисляется автоматически из namespace image topic:

| Image topic | CameraInfo topic |
|---|---|
| `/camera/image_raw` | `/camera/camera_info` |
| `/camera_1/image_raw` | `/camera_1/camera_info` |
| `image_raw` | `image_raw_info` |

Типы сообщений:

| Топик | Тип | Примечание |
|---|---|---|
| `<topic>` | `sensor_msgs/msg/Image` | `bgr8` |
| `<namespace>/camera_info` | `sensor_msgs/msg/CameraInfo` | данные из YAML-калибровки или пустая модель |

Для совместимости с остальным стеком SVERK обычно используется:

```bash
topic:=/camera_1/image_raw
frame_id:=camera_optical_1
```

## Параметры ноды

| Параметр | Тип | По умолчанию | Описание |
|---|---:|---|---|
| `device` | string | `/dev/video8` | V4L2-устройство камеры |
| `topic` | string | `/camera/image_raw` | ROS 2 topic для `sensor_msgs/Image` |
| `fps` | int | `30` | Запрашиваемая частота кадров |
| `in_size` | string | `1280x960` | Разрешение захвата в формате `WIDTHxHEIGHT` |
| `resize_w` | int | `0` в ноде, `320` в launch-файле | Ширина публикуемого кадра; `0` значит как у входа |
| `resize_h` | int | `0` в ноде, `240` в launch-файле | Высота публикуемого кадра; `0` значит как у входа |
| `qos_reliable` | bool | `true` | `true` = RELIABLE, `false` = BEST_EFFORT |
| `calibration_file` | string | `""` | Путь к YAML-файлу калибровки без префикса `file://` |
| `enable_isp` | bool | `true` | Включать Allwinner ISP 3A, если пакет собран с `AWIspApi` |
| `frame_id` | string | `camera_optical` | `frame_id` в заголовках `Image` и `CameraInfo` |

Важно: launch-файл по умолчанию захватывает `1280x960`, но публикует `320x240`, потому что `resize_w:=320` и `resize_h:=240`. Для публикации полного разрешения передай `resize_w:=0 resize_h:=0`.

## Подготовка хоста

На Orange Pi Zero 3W камера должна быть видна ядру как V4L2-устройство. Для IMX219 на A733 используется модуль `vin_v4l2`; отдельный device tree overlay для камеры обычно не нужен, если BSP-ядро уже содержит поддержку камеры.

Проверь модуль и устройства:

```bash
sudo modprobe vin_v4l2
ls -l /dev/video*
v4l2-ctl --list-devices
```

На используемой раскладке устройств:

| Разъем камеры | Устройство |
|---|---|
| CAM 1 | `/dev/video0` |
| CAM 2 | `/dev/video8` |

Чтобы модуль загружался после перезагрузки:

```bash
echo vin_v4l2 | sudo tee -a /etc/modules
```

Пользователь, от которого запускается ROS 2, должен иметь доступ к видео-устройствам:

```bash
sudo usermod -aG video $USER
newgrp video
```

Если используется ISP 3A через `AWIspApi`, библиотека может писать служебные ctx-файлы в `/mnt`. На полетном образе обычно выставляют права так:

```bash
sudo chmod 777 /mnt
sudo chmod 666 /mnt/*.bin 2>/dev/null || true
```

## Зависимости

Установи ROS 2 Humble и системные пакеты:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  v4l-utils libv4l-dev libopencv-dev \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-camera-info-manager \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-ament-cmake
```

Опционально для просмотра изображения:

```bash
sudo apt install -y ros-humble-rqt-image-view
```

Опционально для калибровки:

```bash
sudo apt install -y ros-humble-camera-calibration
```

### Опциональные файлы Allwinner ISP

Для сборки с поддержкой ISP 3A нужны:

```text
/usr/lib/aarch64-linux-gnu/libAWIspApi.so
/usr/lib/aarch64-linux-gnu/libisp.so
/usr/lib/aarch64-linux-gnu/libisp_ini.so
/usr/include/AWIspApi.h
```

`CMakeLists.txt` ищет `AWIspApi.h` в `/usr/include`, `/usr/local/include`, `/usr/include/camera`, а библиотеку в `/usr/lib`, `/usr/local/lib`, `/usr/lib/aarch64-linux-gnu`.

Если `AWIspApi` найден, при сборке будет включен `USE_AWI_SP`. Если не найден, сборка продолжится с предупреждением:

```text
AWIspApi NOT found - ISP 3A disabled
```

## Запуск

### Быстрый старт для CAM 2

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch a733_csi_cam_ros2 camera.launch.py \
  device:=/dev/video8 \
  topic:=/camera_1/image_raw \
  frame_id:=camera_optical_1 \
  in_size:=1280x960 \
  fps:=30 \
  resize_w:=320 \
  resize_h:=240 \
  enable_isp:=true
```

### Полное разрешение без resize

```bash
ros2 launch a733_csi_cam_ros2 camera.launch.py \
  device:=/dev/video8 \
  topic:=/camera_1/image_raw \
  frame_id:=camera_optical_1 \
  in_size:=1280x960 \
  fps:=30 \
  resize_w:=0 \
  resize_h:=0
```

### Запуск CAM 1

```bash
ros2 launch a733_csi_cam_ros2 camera.launch.py \
  device:=/dev/video0 \
  topic:=/camera_0/image_raw \
  frame_id:=camera_optical_0
```

### Запуск без launch-файла

```bash
ros2 run a733_csi_cam_ros2 imx219_camera_node --ros-args \
  -p device:=/dev/video8 \
  -p topic:=/camera_1/image_raw \
  -p frame_id:=camera_optical_1 \
  -p fps:=30 \
  -p in_size:=1280x960 \
  -p resize_w:=320 \
  -p resize_h:=240 \
  -p qos_reliable:=false \
  -p enable_isp:=true
```

## Проверка работы

Список топиков:

```bash
ros2 topic list | grep camera
```

Частота кадров:

```bash
ros2 topic hz /camera_1/image_raw
```

Одно сообщение:

```bash
ros2 topic echo /camera_1/image_raw --once
ros2 topic echo /camera_1/camera_info --once
```

Просмотр изображения:

```bash
ros2 run rqt_image_view rqt_image_view /camera_1/image_raw
```

Ожидаемый лог ноды содержит negotiated format, состояние ISP и статистику FPS каждые 5 секунд:

```text
[V4L2] Format negotiated: 1280x960 planes=1 pixfmt=0x33424752
[ISP] Started isp_id=...
Streaming started with ISP 3A
29.8 fps | frames=149 | ISP=on
```

## Калибровка камеры

В репозитории лежит пример `config/imx219_1280x960.yaml`. Это placeholder с нулевой дисторсией, его нужно заменить реальной калибровкой конкретной камеры и линзы.

Для получения YAML можно использовать стандартный `camera_calibration`:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera_1/image_raw \
  camera:=/camera_1
```

После сохранения калибровки передай путь к файлу:

```bash
ros2 launch a733_csi_cam_ros2 camera.launch.py \
  device:=/dev/video8 \
  topic:=/camera_1/image_raw \
  calibration_file:=$HOME/camera_calibrations/latest_calibration.yaml
```

Если публикуешь изображение после resize, калибровка должна соответствовать выходному разрешению. Например, если `resize_w:=320 resize_h:=240`, используй калибровку для `320x240` либо пересчитай параметры камеры под новое разрешение.

## Docker

Для запуска в контейнере нужно пробросить устройства камеры и POSIX SHM:

```yaml
services:
  ros:
    image: your-image-with-ros2-humble
    ipc: host
    ulimits:
      memlock:
        soft: -1
        hard: -1
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - ROS_LOCALHOST_ONLY=0
      - ISP_CTX_PATH=/mnt
    volumes:
      - /dev:/dev
      - /dev/shm:/dev/shm
      - /mnt:/mnt:rw
```

Если внутри контейнера нужен ISP 3A, добавь в образ или примонтируй Allwinner BSP-библиотеки:

```text
libAWIspApi.so
libisp.so
libisp_ini.so
AWIspApi.h
```

Не ограничивай контейнер только `devices:`, если на твоем BSP-ядре это ломает доступ к vendor-устройствам. Для этой платы рабочий вариант часто проще: пробросить весь `/dev`.

## QoS

По умолчанию launch-файл включает `qos_reliable:=true`. Это удобно для диагностики через стандартные ROS-инструменты, но медленный подписчик может создавать back-pressure.

Для real-time обработки видео часто лучше:

```bash
qos_reliable:=false
```

Тогда image publisher использует `BEST_EFFORT` с глубиной очереди `1`: старые кадры будут отбрасываться, а обработчик будет получать наиболее свежий поток.

## Диагностика V4L2

Проверить, что камера есть:

```bash
v4l2-ctl --list-devices
ls -l /dev/video0 /dev/video8
```

Проверить форматы конкретного устройства:

```bash
v4l2-ctl -d /dev/video8 --list-formats-ext
```

Проверить права:

```bash
groups
ls -l /dev/video8
```

Проверить, что модуль загружен:

```bash
lsmod | grep vin_v4l2
```

## Типовые проблемы

| Симптом | Возможная причина | Что сделать |
|---|---|---|
| `V4L2 open failed` или `Camera init failed` | Неверный `device`, нет прав или модуль `vin_v4l2` не загружен | Проверить `ls /dev/video*`, `sudo modprobe vin_v4l2`, группу `video` |
| Нода стартует, но кадров нет | Камера не подключена, выбран не тот `/dev/video*`, драйвер завис после предыдущего запуска | Проверить шлейф, попробовать CAM 1/CAM 2, перезагрузить плату |
| `S_FMT failed` | Драйвер не поддерживает запрошенное разрешение или BGR24 для этого устройства | Проверить `v4l2-ctl --list-formats-ext`, сменить `in_size` |
| Изображение очень темное | ISP 3A не запустился или пакет собран без `AWIspApi` | Проверить наличие `libAWIspApi.so` и `AWIspApi.h`, запускать с `enable_isp:=true` |
| В логах `ISP init failed` | Нет доступа к ISP ctx-файлам, нет X/окружения BSP или несовместимые vendor-библиотеки | Проверить права на `/mnt`, библиотеки BSP, переменные окружения образа |
| FPS ниже ожидаемого | Слишком большое разрешение, RELIABLE QoS тормозит из-за подписчика, CPU governor не performance | Включить `qos_reliable:=false`, уменьшить `resize`, проверить CPU governor |
| `CameraInfo` публикуется, но параметры нулевые | Не передан реальный `calibration_file` | Сделать калибровку и передать путь к YAML |
| `SCHED_FIFO failed` | Нет root или `CAP_SYS_NICE` | Это не фатально; для минимальной задержки запускать с нужными capabilities |

## Файлы репозитория

Для самостоятельного репозитория уже подготовлены:

- `ABOUT.md` - короткое описание, topics и tagline для GitHub About;
- `LICENSE` - MIT license;
- `.gitignore` - ROS 2, colcon, CMake, editor/cache/runtime artifacts;
- `CHANGELOG.md` - история изменений;
- `CONTRIBUTING.md` - короткие правила разработки и проверки;
- `.github/workflows/ci.yml` - базовая CI-сборка ROS 2 Humble на Ubuntu 22.04.

## Лицензия

MIT. См. `LICENSE`.
