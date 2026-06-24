# Работа с моделями
## PSPSO
PSPSO - метод оптимизации, используемый для подбора коэффициентов
## Neural models
В директории лежат директории с обученными моделями одного базиса, блокнотом,
на котором проходило обучение базиса, и файл со структурой сети,
как из блокнота, чтобы получить функции из нейронных моделей.
## Тестирование
Для тестирования моделей используется скрипт model_test.py.
Ему нужно передать название директории с базисом из `src/model/neural_models/`
и название одиночного CSV-датасета из `datasets/`.
Перед запуском нужно собрать PSPSO-модуль, потому что `model_test.py`
импортирует `pspso` из `src/model/PSPSO/build`.

```bash
source src/scripts/python_venv_setup.sh
src/scripts/build_pspso.sh
python src/model/model_test.py model_example voronoi_example
```

То же самое через оболочку запуска:

```bash
src/scripts/start_model_testing.sh model_example voronoi_example
```

Скрипт читает все `.pt` и `.pth` файлы из
`src/model/neural_models/<basis_name>/`, достаёт из них `state_dict`
для `ODEFunc`, нумерует модели при выводе графиков и считает one-step MAE
по `odom_vx` и `odom_wz`.
