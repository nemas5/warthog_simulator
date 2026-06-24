from __future__ import annotations

import random
import json
import math
from string import Template
from pathlib import Path


class Surface:
    # Шаблон с описанием свойств поверхности
    _template: Template | None = None
    _template_file = Path(__file__).resolve().parent / "templates" / "template.world"

    # По умолчанию параметры для "асфальта"
    def __init__(
            self, name: str, friction_mu: float = 0.9, friction_mu2: float = 0.9, friction_slip1: float = 0.0,
            friction_slip2: float = 0.0, torsional_coefficient: float = 0.03,
            surface_radius: float = 0.12, torsional_slip: float = 0.0,
            bounce_coefficient: float = 0.0, bounce_threshold: float = 0.1,
            contact_kp: float = 5e5, contact_kd: float = 50.0,
            visual_rgba: str | None = None
        ):
        self.name = name

        self.friction_mu = friction_mu
        self.friction_mu2 = friction_mu2
        self.friction_slip1 = friction_slip1
        self.friction_slip2 = friction_slip2

        self.torsional_coefficient = torsional_coefficient
        self.torsional_surface_radius = surface_radius  # Радиус колеса
        self.torsional_slip = torsional_slip

        self.bounce_coefficient = bounce_coefficient
        self.bounce_threshold = bounce_threshold

        self.contact_kp = contact_kp
        self.contact_kd = contact_kd

        # Задаётся случайный цвет, если он не указан
        if visual_rgba is None:
            visual_rgba = f"{random.random()} {random.random()} {random.random()} 1"
        self.visual_rgba = visual_rgba

        # Хранит подстановку всех параметров физики и отображения, оставляет геометрию незаполненной
        self.xml = self._to_xml()

    # Читает файл только в первый раз для всех экземпляров, потом возвращает ранее прочитанный шаблон
    @classmethod
    def _get_template(cls) -> Template:
        if cls._template is None:
            with cls._template_file.open("r") as f:
                cls._template = Template(f.read())
        return cls._template

    # geometry не подставляется, она вставляется на уровне создания мира
    def _to_xml(self) -> Template:
        return Template(
            self._get_template().safe_substitute(
                friction_mu=self.friction_mu,
                friction_mu2=self.friction_mu2,
                friction_slip1=self.friction_slip1,
                friction_slip2=self.friction_slip2,
                torsional_coefficient=self.torsional_coefficient,
                torsional_surface_radius=self.torsional_surface_radius,
                torsional_slip=self.torsional_slip,
                bounce_coefficient=self.bounce_coefficient,
                bounce_threshold=self.bounce_threshold,
                contact_kp=self.contact_kp,
                contact_kd=self.contact_kd,
                visual_rgba=self.visual_rgba,
            )
        )

    # geomerty раздаётся генератором карты, возвращает шаблон фрагмента поверхности
    def getTile(self, geometry: str) -> str:
        return self.xml.safe_substitute(geometry=geometry)
    
    def getName(self) -> str:
        return self.name


class World:
    # Шаблон с описанием общих параметров мира
    _template: Template | None = None
    # Директория сохранения миров
    _worlds_dir_name = "worlds_library"
    _template_file = Path(__file__).resolve().parent / "templates" / "world_header_template.world"
    _package_dir = Path(__file__).resolve().parent.parent
    _default_worlds_dir = _package_dir / _worlds_dir_name

    def __init__(
            self, surface_types: list, world_name: str = "plate",
            solver_iterations: int = 50, max_step_size: float = 0.004,
            real_time_update_rate: int = 250,
            worlds_dir: str | Path | None = None
        ):
        self.surface_types = surface_types  # Список поверхностей, используемых при генерации мира
        self.world_name = world_name
        self._worlds_dir = Path(worlds_dir).expanduser().resolve() if worlds_dir else self._default_worlds_dir
        self._meshes_dir = self._worlds_dir / world_name / "meshes"
        self.solver_iterations = solver_iterations
        self.max_step_size = max_step_size
        self.real_time_update_rate = real_time_update_rate

    # Читает файл шаблона хедера только в первый раз для всех экземпляров, потом возвращает ранее прочитанный шаблон
    @classmethod
    def _get_template(cls) -> Template:
        if cls._template is None:
            with cls._template_file.open("r") as f:
                cls._template = Template(f.read())
        return cls._template

    # Подстановка параметров мира в шаблон
    def _to_xml(self) -> str:
        return self._get_template().safe_substitute(
            world_name=self.world_name,
            solver_iterations=self.solver_iterations,
            max_step_size=self.max_step_size,
            real_time_update_rate=self.real_time_update_rate,
        )

    # Создание plate - бесконечной поверхности из одного материала
    def generate_one_surface_world(self, ind: int = 0) -> None:
        if ind < 0 or ind >= len(self.surface_types):
            raise IndexError("surface index is out of range")
        header = self._to_xml()
        geometry = """
            <plane>
            <normal>0 0 1</normal>
            <size>1000 1000</size>
            </plane>
        """.rstrip()
        surface = self.surface_types[ind]
        surface_xml = surface.getTile(geometry)
        model_xml = f"""
            <model name="{self.world_name}_surface">
                <static>true</static>
                <pose>0 0 0 0 0 0</pose>
            {surface_xml}
            </model>
        """.rstrip()
        map_json = {
            "world_name": self.world_name,
            "map_type": "single_surface",
            "surface_name": surface.getName(),
            "surface_names": [surface.getName()],
        }
        self._generate_world_file(header + "\n" + model_xml, map_json)
        return None

    # Мир, случайно замощенный квадратами каждого из типов поверхностей
    def generate_squares_world(
            self, tile_size: float, 
            map_width: float, 
            map_lenght: float,
            distribution: list | None = None
        ) -> None:
        import numpy as np

        map_length = map_lenght
        distribution = self._normalized_distribution(distribution)
        if tile_size <= 0:
            raise ValueError("tile_size must be positive")

        header = self._to_xml()
        tiles = []
        height = 1.

        tiles_x = map_width / tile_size
        tiles_y = map_length / tile_size
        if not math.isclose(tiles_x, round(tiles_x)) or not math.isclose(tiles_y, round(tiles_y)):
            raise ValueError("map_width and map_length must be divisible by tile_size")
        number_of_tiles_x = int(round(tiles_x))
        number_of_tiles_y = int(round(tiles_y))
        if number_of_tiles_x <= 0 or number_of_tiles_y <= 0:
            raise ValueError("map dimensions must contain at least one tile")

        map_of_surface_types = np.random.choice(
            len(self.surface_types), 
            size=(number_of_tiles_y, number_of_tiles_x),
            p=distribution
        )
        map_center_x = map_width / 2.0
        map_center_y = map_lenght / 2.0

        for j in range(number_of_tiles_y):
            for i in range(number_of_tiles_x):
                surface_type_ind = int(map_of_surface_types[j, i])
                current_surface_type = self.surface_types[surface_type_ind]

                center_x = -map_center_x + (i + 0.5) * tile_size
                center_y = -map_center_y + (j + 0.5) * tile_size

                geometry = f"""
                    <box>
                    <size>{tile_size} {tile_size} {height}</size>
                    </box>
                """.rstrip()
                surface_xml = current_surface_type.getTile(geometry)
                model_xml = f"""
                    <model name="tile_{i}_{j}">
                        <static>true</static>
                        <pose>{center_x:.3f} {center_y:.3f} {-height / 2:.3f} 0 0 0</pose>
                        {surface_xml}
                    </model>
                    """.rstrip()
                tiles.append(model_xml)
        map_json = {
            "world_name": self.world_name,
            "map_type": "squares",
            "tile_size": tile_size,
            "map_width": map_width,
            "map_length": map_length,
            "rows": number_of_tiles_y,
            "cols": number_of_tiles_x,
            "origin": {"x": -map_center_x, "y": -map_center_y},
            "bounds": {
                "min_x": -map_center_x,
                "max_x": map_center_x,
                "min_y": -map_center_y,
                "max_y": map_center_y,
            },
            "surface_names": [surface.getName() for surface in self.surface_types],
            "map": [[self.surface_types[tile].getName() for tile in row] for row in map_of_surface_types]
        }
        self._generate_world_file(header + "\n" + "\n".join(tiles), map_json)
        return None

    # Мир, случайно замощенный диаграммой Вороного
    def generate_voronoi_world(
            self,
            map_width: float,
            map_lenght: float,
            number_of_points: int,
            distribution: list | None = None
        ) -> None:
        import numpy as np
        import trimesh
        from shapely.geometry import MultiPoint, box as shapely_box, mapping
        from shapely.ops import voronoi_diagram

        if number_of_points <= 0:
            raise ValueError("number_of_points must be positive")
        distribution = self._normalized_distribution(distribution)

        header = self._to_xml()
        tiles = []
        height = 1

        map_center_x = map_width / 2.0
        map_center_y = map_lenght / 2.0

        random_generator = np.random.default_rng()
        centers = random_generator.uniform(
            low=[-map_center_x, -map_center_y],
            high=[map_center_x, map_center_y],
            size=(number_of_points, 2)
        )  # Случайные координаты точек внутри карты

        # Прямоугольный полигон карты
        map_polygon = shapely_box(-map_center_x, -map_center_y, map_center_x, map_center_y)
        # type(vor) == GeometryCollection - контейнер Polygon, центров больше нет
        vor = voronoi_diagram(MultiPoint(centers), envelope=map_polygon)
        surface_map = []
        clipped_cells = []
        # Создание директории для мешей (и для мира, если её ещё нет)
        self._meshes_dir.mkdir(parents=True, exist_ok=True)
        for old_mesh in self._meshes_dir.glob("*.stl"):
            old_mesh.unlink()

        for raw_poly in vor.geoms:
            # Выдача точке типа в зависимости от распределения
            surface_type_index = random_generator.choice(
                len(self.surface_types),
                p=distribution
            ) 
            surf = self.surface_types[surface_type_index]
            surface_name = self.surface_types[surface_type_index].getName()

            clipped = raw_poly.intersection(map_polygon)
            for poly in self._iter_polygons(clipped):
                if poly.is_empty or poly.area <= 1e-9:
                    continue
                zone_index = len(clipped_cells)
                name = f"{self.world_name}_zone_{zone_index}"
                clipped_cells.append(poly)
                surface_map.append(surface_name)

                # Создание mesh из Polygon, который можно будет использовать в качестве геометрии в .world
                mesh = trimesh.creation.extrude_polygon(poly, height=height)
                mesh_path = self._meshes_dir / f"{name}.stl"
                mesh.export(mesh_path)

                # Генерация xml для фрагмента поверхности в .world
                geometry = f"""
                    <mesh>
                    <uri>{self._mesh_uri(mesh_path)}</uri>
                    </mesh>
                """.rstrip()
                surface_xml = surf.getTile(geometry)
                model_xml = f"""
                    <model name="{name}">
                        <static>true</static>
                        <pose>0 0 {-height / 2.0:.3f} 0 0 0</pose>
                        {surface_xml}
                    </model>
                """.rstrip()
                tiles.append(model_xml)

        if not clipped_cells:
            raise RuntimeError("Voronoi generation produced no cells inside map bounds")

        map_file = {
            "world_name": self.world_name,
            "map_type": "voronoi",
            "map_width": map_width,
            "map_length": map_lenght,
            "origin": {"x": -map_center_x, "y": -map_center_y},
            "bounds": {
                "min_x": -map_center_x,
                "max_x": map_center_x,
                "min_y": -map_center_y,
                "max_y": map_center_y,
            },
            "surface_names": [surface.getName() for surface in self.surface_types],
            "map": [mapping(g) for g in clipped_cells],
            "surface_map": surface_map
        }  # Записываем регионы в словарь, в том же порядке каждому региону даётся его повернхость

        self._generate_world_file(header + "\n" + "\n".join(tiles), map_file)
        return None

    def _mesh_uri(self, mesh_path: Path) -> str:
        try:
            relative_mesh_path = mesh_path.resolve().relative_to(self._package_dir)
            return f"model://{relative_mesh_path.as_posix()}"
        except ValueError:
            return mesh_path.resolve().as_uri()

    def _normalized_distribution(self, distribution: list | None) -> list[float]:
        if not self.surface_types:
            raise ValueError("at least one surface type is required")
        if distribution is None:
            return [1.0 / len(self.surface_types)] * len(self.surface_types)
        probabilities = [float(value) for value in distribution]
        if len(probabilities) != len(self.surface_types):
            raise ValueError("distribution length must match number of surface types")
        if any(value < 0.0 for value in probabilities):
            raise ValueError("distribution probabilities must be non-negative")
        total = sum(probabilities)
        if total <= 0.0:
            raise ValueError("distribution sum must be positive")
        return [value / total for value in probabilities]

    def _iter_polygons(self, geometry):
        if geometry.geom_type == "Polygon":
            yield geometry
        elif geometry.geom_type in ("MultiPolygon", "GeometryCollection"):
            for item in geometry.geoms:
                yield from self._iter_polygons(item)

    # Запись в файлы сгенерированного мира
    def _generate_world_file(self, world_xml: str, world_surface_type_map: dict | None) -> None:
        full_xml = world_xml.rstrip() + "\n  </world>\n</sdf>\n"  # Добавляются закрывающие теги для заголовка
        world_fname = f"{self.world_name}.world"
        world_dir = self._worlds_dir / self.world_name
        world_dir.mkdir(parents=True, exist_ok=True)
        file_path = world_dir / world_fname
        with file_path.open("w") as f:
            f.write(full_xml)

        if world_surface_type_map is not None:
            file_path = world_dir / f"{self.world_name}_surface_map.json"
            with file_path.open("w") as file:
                json.dump(world_surface_type_map, file, indent=2)


def generate_basis_worlds(
        surfaces: list[Surface],
        world_set_name: str = "basis_example"
    ) -> Path:
    basis_worlds_dir = World._default_worlds_dir / world_set_name
    for surface in surfaces:
        World([surface], surface.getName(), worlds_dir=basis_worlds_dir).generate_one_surface_world()
    return basis_worlds_dir


if __name__ == "__main__":
    # Для создание своего мира необходимо поменять Surface в 
    # списке surfaces, подставив свои параметры и количество поверхностей
    surfaces = [
        Surface(
            "ice",
            friction_mu=0.03,
            friction_mu2=0.02,
            friction_slip1=0.03,
            friction_slip2=0.03,
            torsional_coefficient=0.005,
            surface_radius=0.12,
            torsional_slip=0.01,
            contact_kp=1e7,
            contact_kd=1e3,
            visual_rgba="0.55 0.80 1.00 1",
        ),
        Surface(
            "wet_asphalt",
            friction_mu=0.4,
            friction_mu2=0.15,
            contact_kp=1e9,
            contact_kd=1e6,
            visual_rgba="0.12 0.12 0.12 1",
        ),
        Surface(
            "dry_concrete",
            friction_mu=0.8,
            friction_mu2=0.3,
            contact_kp=1e10,
            contact_kd=1e7,
            visual_rgba="0.55 0.55 0.55 1",
        ),
        Surface(
            "mud",
            friction_mu=0.6,
            friction_mu2=0.4,
            friction_slip1=0.08,
            friction_slip2=0.08,
            contact_kp=1e6,
            contact_kd=1e4,
            visual_rgba="0.30 0.20 0.10 1",
        ),
        Surface(
            "soft_soil",
            friction_mu=0.5,
            friction_mu2=0.5,
            friction_slip1=0.05,
            friction_slip2=0.05,
            contact_kp=1e5,
            contact_kd=1e4,
            visual_rgba="0.36 0.28 0.16 1",
        )
    ]
    basis_name = "basis_example"
    basis_worlds_dir = generate_basis_worlds(surfaces, basis_name)

    # Пример генерации мира, заполненного случайной сеткой поверхностей
    World(surfaces, "squares_example").generate_squares_world(
        tile_size=8.0,
        map_width=64.0,
        map_lenght=64.0,
    )
    # Пример генерации мира, заполненного случайным распределением поверхностей
    # по диаграмме Вороного
    World(surfaces, "voronoi_example").generate_voronoi_world(
        map_width=64.0,
        map_lenght=64.0,
        number_of_points=20,
    )
