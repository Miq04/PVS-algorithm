from dataclasses import dataclass

@dataclass(frozen=True)
class LiDAR:
    name: str
    num_channels: int
    range_min: float
    range_max: float
    horizontal_fov: float
    horizontal_resolution: int
    vertical_fov: tuple
    vertical_resolution: float
    frequency: float

class LiDARTypes:
    Pandar64 = LiDAR(
        name="Pandar64",
        num_channels=64,
        range_min=0.3,
        range_max=200,
        horizontal_fov=360,
        horizontal_resolution=0.2,
        vertical_fov=(-25, 15),
        vertical_resolution=0.167,
        frequency=10
    )
    PandarQT = LiDAR(
        name="PandarQT",
        num_channels=64,
        range_min=0.1,
        range_max=60,
        horizontal_fov=360,
        horizontal_resolution=0.6,
        vertical_fov=(-52.1,52.1),
        vertical_resolution=1.45,
        frequency=10
    )
    Test = LiDAR(
        name="Test",
        num_channels=32,
        range_min=0.1,
        range_max=60,
        horizontal_fov=360,
        horizontal_resolution=10,
        vertical_fov=(-25,15),
        vertical_resolution=5,
        frequency=10
    )
    Test1 = LiDAR(
        name="Inno300",
        num_channels=1,
        range_min=0.1,
        range_max=50,
        horizontal_fov=360,
        horizontal_resolution=1,
        vertical_fov=(-25,15),
        vertical_resolution=1,
        frequency=32
    )
