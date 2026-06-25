# Симулятор движения Warthog в Gazebo

Симулятор предназначен для тестирования моделей динамики колёсных роботов и оценки качества автономного управления.

Возможности симулятора:

- Генерация плоских миров из заданного набора поверхностей с конфигурируемыми свойствами;
- Симуляция движения колесного робота по заданной функцией траектории в сгенерированном мире;
- Сбор одометрии во время симуляции;
- Преобразование одометрии в датасеты для обучения нейронных сетей динамике движения по сгенерированным поверхностям;
- Оценка точности предсказаний модели на основе данных симуляции;
- Симуляция управления движением на основе MPC и оценка качества управления.

## Установка

### ROS1 Noetic

Для установки симулятора требуется ROS1 Noetic с Gazebo.
Может быть установлен как Docker-контейнер ([скачать образ](https://hub.docker.com/layers/osrf/ros/noetic-desktop-full/images/sha256-bfb0effabc17db413c112b4aa368a11918fd84aa9470e830b044d7ce72e84f19)).

Пример конфигурации `docker run` для запуска на Ubuntu:
```bash
xhost +local:root
sudo docker run -it \
  --name ros-noetic-gazebo \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/ros-noetic":/ros-noetic \
  osrf/ros:noetic-desktop-full
```

### Установка симулятора

Перейти в директорию с кодом проекта:
```bash
cd ~/ros-noetic/warthog_simulator
```
Обновить индексы APT и установить системные и ROS-зависимости, включая Warthog, `catkin build`, `python3-pip` и `python3-venv`:
```bash
src/scripts/setup.sh
```
Подключить базовое окружение ROS Noetic в текущем терминале:
```bash
source /opt/ros/noetic/setup.bash
```
Установить остальные ROS/system-зависимости из `package.xml`:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
Создать Python-окружение и поставить зависимости для ROS-нод, инструментов генерации данных и обработки результатов
симуляций из `requirements.txt`:
```bash
source src/scripts/python_venv_setup.sh
```
После изменения `requirements.txt` лучше заново выполнить `source src/scripts/python_venv_setup.sh` и пересобрать пакет, чтобы `catkin_install_python` использовал актуальный Python.

Собрать catkin-пакет:
```bash
catkin build warthog_simulator
```
Обновить зависимости:
```bash
source devel/setup.bash
```
После проделанных шагов симулятор готов к использованию по инструккциям ниже.

Сборка python-библиотеки PSPSO: `model_test.py` импортирует `pspso` из `src/model/PSPSO/build`. Версия в репозитории собрана под Python 3.8, стандартный для контейнера с ROS1 Noetic. При использовании другой версии или изменении в алгоритме оптимизации нужно пересобрать библиотеку с помощью скрипта:
```bash
src/scripts/build_pspso.sh
```

## Структура проекта

Скрипты используют фиксированную структуру каталогов относительно корня workspace. Имена `src`, `bags`, `datasets`, `model` и вложенные пути ниже являются частью интерфейса проекта: простое перемещение файлов или переименование директорий нарушит поиск входных данных и сохранение результатов.

```text
warthog_simulator/                 # корень catkin workspace
├── src/                           # пакет warthog_simulator
│   ├── scripts/                   # оболочки запуска и настройки окружения
│   │   ├── setup.sh
│   │   ├── python_venv_setup.sh
│   │   └── start_simulation.sh
│   ├── ros_nodes/                 # ROS-ноды симуляции
│   │   ├── trajectory_runner.py
│   │   └── surface_detector.py
│   ├── bag_parser/                # парсер bag в csv
│   │   └── gazebo_dataset_parser.py
│   ├── world_generation/          # генератор миров и его SDF-шаблоны
│   ├── worlds_library/            # библиотека миров — вход start_simulation.sh
│   │   ├── <world_name>/          # одиночный мир
│   │   │   ├── <world_name>.world
│   │   │   ├── <world_name>_surface_map.json
│   │   │   └── meshes/            # STL для сложной геометрии, если нужны
│   │   └── <world_set_name>/      # набор однородных миров
│   │       └── <surface_name>/
│   │           ├── <surface_name>.world
│   │           └── <surface_name>_surface_map.json
│   ├── urdf/                      # дополнительные Xacro-компоненты Warthog
│   ├── model/                     # код и файлы моделей
│   │   ├── model_test.py
|   |   ├── PSPSO/                 # собранный пакет метода оптимизации для теста моделей
│   │   └── neural_models/         # сюда помещаются сохранённые модели
│   ├── simulation.launch
│   ├── package.xml
│   └── requirements.txt
├── bags/                          # rosbag, создаваемые симуляцией
│   ├── <world_name>.bag           # результат одиночного мира
│   └── <world_set_name>/          # результаты набора миров
│       └── <surface>_<run_id>.bag
├── datasets/                      # CSV, создаваемые bag-парсером
│   ├── <world_name>.csv
│   └── <world_set_name>/
│       └── <surface>_<run_id>.csv
├── PSPSO/                         # C++ библиотека для Python с методом оптимизации (используется в модели)
|
├── neural_ode_notebook/           # Ноутбук для обучения моделей
|
|
├── devel/                         # catkin build space (генерируется при сборке проекта)
|
├── build/                         # catkin build space (генерируется при сборке проекта)
|
├── logs/                          # catkin build space (генерируется при сборке проекта)
```

Основные привязки путей:

| Данные | Откуда читаются | Как записываются |
|--------|-----------------|--------------------|
| Миры | `src/worlds_library/` | Генератор также пишет в `src/worlds_library/`. |
| Карты поверхностей | Рядом с `.world`, файл `*_surface_map.json` | Создаются вместе с миром. |
| STL-меши мира | `<world_name>/meshes/` | Создаются генератором для мира Вороного. |
| Bag-файлы | `bags/<world>.bag` или `bags/<set>/` | При симуляции записываются топики специальной нодой. |
| CSV-датасеты | `datasets/<world>.csv` или `datasets/<set>/` | записываются парсером после обработки `bags/` |
| Модели | `src/model/neural_models/` | Сюда нужно помещать модель для последующего тестирования. |
| URDF/Xacro-расширения | `src/urdf/` | Подключаются явным путём из `simulation.launch`. |

`start_simulation.sh` находит корень workspace относительно собственного расположения в `src/scripts/`, а `gazebo_dataset_parser.py` ищет каталог, содержащий `src/package.xml`. Поэтому запуск возможен из разных текущих директорий, но сама структура workspace должна оставаться неизменной.

## Генерация миров

Для генерации мира используется утилита `world_generator.py`.
Конфигурация генерируемого мира задаётся в блоке `if __name__ == "__main__"`. Настраиваемые параметры:

- Имя генерируемого мира;
- Параметры поверхностей, подсказка по коэффициентам находится в `world_generator/README.md`;
- Количество поверхностей;
- Способ замощения мира различными поверхностями (квадратная сетка, диаграмма Вороного, бесконечный мир из одной поверхности);
- Количество генерируемых миров.

Инструмент даёт возможность собрать набор `.bag` и `.csv` для движения Warthog по базису поверхностей. Для этого генератор создаёт директорию набора миров, где каждый мир является бесконечной плоскостью с одной поверхностью.

Запуск генерации:

```bash
cd ~/ros-noetic/warthog_simulator
source src/scripts/python_venv_setup.sh
python src/world_generation/world_generator.py
```

После генерации набора однородных поверхностей в библиотеке новых миров будет создана отдельная директория:

```text
src/worlds_library/basis_example/
  ice/ice.world
  ice/ice_surface_map.json
  wet_asphalt/wet_asphalt.world
  wet_asphalt/wet_asphalt_surface_map.json
  ...
```
В случае генерации одного мира с помощью диаграммы Вороного:

```text
src/worlds_library/ice
  meshes/
    ice_zone_0.stl
    ice_zone_1.stl
    ...
  ice.world
  ice_surface_map.json
```
Карты формата `*_surface_map.json` позволяют определить по положению робота поверхность под ним с помощью `surface_detector.py`.
Инструмент поддерживает карты `single_surface`, `squares` и `voronoi`.

## Запуск симуляции для сбора данных (тестовых или для обучения)

`start_simulation.sh` принимает имя одиночного мира или набора и определяет режим по структуре `src/worlds_library/`:

```text
<name>/<name>.world          # одиночный мир
<name>/<surface>/<surface>.world  # набор миров
```

```bash
# Один мир
src/scripts/start_simulation.sh world_example

# Набор миров, короткий прогон без Gazebo UI
src/scripts/start_simulation.sh basis_example --duration-sec 300 --trajectory-type circle
```

Для каждого мира запускаются `trajectory_runner.py`, `surface_detector.py` и `rosbag record`, затем bag автоматически преобразуется в CSV:

- одиночный мир: `bags/<name>.bag` и `datasets/<name>.csv`;
- набор: bag и CSV каждого мира в `bags/<name>/` и `datasets/<name>/`.

Основные параметры:

| Параметр | По умолчанию | Назначение |
|----------|--------------|------------|
| `--x`, `--y`, `--z` | `0.0`, `0.0`, `0.5` | Начальная позиция Warthog. |
| `--gui` | выключен | Показать интерфейс Gazebo. |
| `--trajectory-type NAME` | `circle` | `circle`, `segments` или `interpolated`. |
| `--duration-sec VALUE` | `1800` | Длительность прогона каждого мира, с. |
| `--rate VALUE` | `20` | Частота публикации команд, Гц. |
| `--no-parse` | выключен | Не преобразовывать bag в CSV. |

Доступные типы траекторий в базовом `trajectory_runner.py`:
- `circle` — аналитическое движение по окружности;
- `segments` — дискретная последовательность команд `(linear_x, angular_z, duration)`;
- `interpolated` — линейная интерполяция между ключевыми командами `(time, linear_x, angular_z)`.

## Кастомизация траектории движения

`trajectory_runner.py` подписывается на `/ground_truth/odom`, рассчитывает команду движения с заданной частотой и публикует `geometry_msgs/Twist` в `/cmd_vel`. Перед началом движения нода ждёт первое сообщение одометрии.

Каждая траектория реализует общий интерфейс:

```python
class Trajectory:
    def command(self, elapsed: float, pose: Pose2D) -> VelocityCommand:
        ...
```

- `elapsed` — время, прошедшее с начала траектории;
- `pose` — текущие `x`, `y` и `yaw` робота;
- результат — команда `VelocityCommand(linear_x, angular_z)`.

Созданная команда ограничивается значениями `max_linear_speed` и `max_angular_speed`, после чего публикуется в `/cmd_vel`.

### Регистрация типа траектории

Строка из параметра `--trajectory-type` проходит по цепочке:

```text
start_simulation.sh
  → simulation.launch
  → параметр ~trajectory_type ноды trajectory_runner
  → TrajectoryFactory.create()
```

Фабрика заполняется в методе `TrajectoryRunner._build_factory()`:

```python
factory.register("circle", self._build_circle)
```

Чтобы добавить новый тип:

1. Реализовать класс-наследник `Trajectory` или builder, возвращающий `AnalyticalTrajectory`.
2. Добавить builder в `TrajectoryRunner`.
3. Зарегистрировать его строковое имя в `_build_factory()`:

   ```python
   factory.register("my_trajectory", self._build_my_trajectory)
   ```

4. Запустить симуляцию с тем же именем:

   ```bash
   src/scripts/start_simulation.sh world_example --trajectory-type my_trajectory
   ```

Пример простой аналитической траектории:

```python
def _build_my_trajectory(self) -> Trajectory:
    def command(elapsed: float, pose: Pose2D) -> VelocityCommand:
        linear_x = 0.5
        angular_z = 0.4 * math.sin(elapsed)
        return VelocityCommand(linear_x, angular_z)

    return AnalyticalTrajectory("my_trajectory", command)
```

Для траектории с внутренним состоянием удобнее создать отдельный класс. Если нужны новые настраиваемые параметры, их следует добавить в `RunnerParams` и загрузить в `_load_params()`.

## Обучение и добавление модели для тестирования

Модели для тестирования могут быть обучены и загружены в директорию `src/model/neural_models/<models_set>`. При тестировании дириектория `<models_set>` указывается в вызове `start_model_testing`. `model_test.py` автоматически заберёт все модели из диреткории, проинициализирует их для использования и протестирует алгоритм.

## Запуск тестирования модели

`model_test.py` принимает название директории базиса из `src/model/neural_models/`
и название одиночного CSV-датасета из `datasets/`. Вложенные директории внутри
`datasets/` этим тестом не обрабатываются. Перед запуском теста должен быть
собран PSPSO-модуль:

```bash
source src/scripts/python_venv_setup.sh
src/scripts/build_pspso.sh
python src/model/model_test.py model_example voronoi_example
```

Или через небольшой скрипт, который сам подключает Python-окружение:

```bash
src/scripts/start_model_testing.sh model_example voronoi_example
```

Для примера выше будут прочитаны модели из
`src/model/neural_models/model_example/` и датасет `datasets/voronoi_example.csv`.

## Запуск тестирования управления на основе модели

В разработке.

## Записываемые в bag-файл топики

| Топик                   | Тип топика            | Описание                                                                    |
|-------------------------|-----------------------|-----------------------------------------------------------------------------|
| `/ground_truth/odom`    | `nav_msgs/Odometry`   | Точное положение из Gazebo                                                  |
| `/odometry/filtered`    | `nav_msgs/Odometry`   | EKF-одометрия (Extended Kalman Filter) (для линейной и угловой скоростей)   |
| `/terrain/surface_type` | `std_msgs/String`     | Строка-тип поверхности под роботом на основе реального положения            |
| `/cmd_vel`              | `geometry_msgs/Twist` | Команды скорости (угловой и линейной)                                       |
| `/imu/data`             | `sensor_msgs/Imu`     | IMU (для угловой скорости)                                                  |

## Парсинг bag в датасет

Обычно парсер запускает `start_simulation.sh`. Вручную его можно вызвать по имени мира или набора:

```bash
source src/scripts/python_venv_setup.sh
src/bag_parser/gazebo_dataset_parser.py basis_example
```

Для одиночного мира парсер обрабатывает `bags/<name>.bag`. Для набора он читает все `.bag` из `bags/<name>/` и сохраняет CSV с теми же базовыми именами в `datasets/<name>/`.

Парсер объединяет `/terrain/surface_type`, `/cmd_vel` и одометрию по ближайшему времени.

## Расширение модели Warthog через URDF/Xacro

Дополнительные компоненты робота — лидары, камеры, датчики и Gazebo-плагины — можно добавлять отдельными файлами в `src/urdf/`. Warthog не подключает всю директорию автоматически: каждый дополнительный Xacro-файл нужно явно передать через переменную окружения до запуска `roslaunch`.

Текущий `start_simulation.sh` подключает точную одометрию перед вызовом `simulation.launch`:

```bash
export WARTHOG_URDF_EXTRAS="${PACKAGE_DIR}/urdf/warthog_ground_truth.urdf.xacro"
```

Для нового компонента следует:

1. Создать отдельный файл, например `src/urdf/warthog_lidar.urdf.xacro`.
2. Описать в нём `link`, фиксированный `joint` к раме Warthog и Gazebo sensor/plugin.
3. Экспортировать в `start_simulation.sh` переменную окружения, которую поддерживает установленное описание Warthog или подключаемый Xacro-файл:

   ```bash
   export WARTHOG_LIDAR_URDF="${PACKAGE_DIR}/urdf/warthog_lidar.urdf.xacro"
   ```

4. Явно включить этот файл из основного файла расширений или из другого Xacro, который уже передан через `WARTHOG_URDF_EXTRAS`:

   ```xml
   <xacro:include filename="$(optenv WARTHOG_LIDAR_URDF)"/>
   ```

Пример упрощённого компонента:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="front_lidar_link"/>

  <joint name="front_lidar_joint" type="fixed">
    <parent link="top_chassis_link"/>
    <child link="front_lidar_link"/>
    <origin xyz="0.5 0 0.6" rpy="0 0 0"/>
  </joint>

  <gazebo reference="front_lidar_link">
    <sensor name="front_lidar" type="ray">
      <update_rate>10</update_rate>
      <!-- Параметры сканирования и Gazebo ROS plugin. -->
    </sensor>
  </gazebo>
</robot>
```

При добавлении компонента также необходимо:

- добавить ROS/Gazebo-пакет его плагина в `package.xml`;
- при появлении новых Python-зависимостей добавить их в `requirements.txt`;
- убедиться, что Xacro/mesh-файлы устанавливаются через `CMakeLists.txt` — текущая директория `urdf` уже устанавливается целиком;
- добавить новые ROS-топики в `rosbag record` внутри `simulation.launch`, если их нужно сохранять;
- расширить `gazebo_dataset_parser.py`, если данные нового сенсора должны попадать в CSV.

После изменения `package.xml`, URDF/Xacro или состава зависимостей нужно повторно выполнить:

```bash
src/scripts/setup.sh
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
source src/scripts/python_venv_setup.sh
catkin build warthog_simulator
source devel/setup.bash
```
