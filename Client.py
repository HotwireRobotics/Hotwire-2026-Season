from enum import Enum
import time, pygame as pg
import pygame_gui as pgui
import networktables as nt
import os, sys
from networktables import NetworkTables as NT, NetworkTable

base: str = os.path.dirname(os.path.abspath(__file__))
deploy: str = os.path.join(base, "src", "main", "deploy")

SERVER: str = "10.29.90.2"
FPS: int = 60
RUNNING: bool = True
NT.initialize(server=SERVER)

table: NetworkTable  = NT.getTable("AdvantageKit")
driverStation = table.getSubTable("DriverStation")
outputs         = table.getSubTable("RealOutputs")

pg.init()

field: pg.Surface = pg.image.load(os.path.join(base, "assets", "field.png"))
field_render = field.copy()
size: tuple = field.get_size()

pg.display.set_caption(f"Firelight - {SERVER}")
pg.display.set_icon(pg.image.load(os.path.join(deploy, "logo.png")))
manager: pgui.UIManager = pgui.UIManager(size)
clock: pg.Clock = pg.Clock()

screen: pg.Surface = pg.display.set_mode(size, pg.RESIZABLE|pg.SRCALPHA)

# label = pgui.elements.UILabel(
#     relative_rect=pg.Rect(5, 5, 50, 50),
#     text="Hello",
#     manager=manager
# )

class State(Enum):
    REAL: int = None
    SIM:  int = None
state: State = State.REAL


while RUNNING:
    dtime: float = clock.tick(FPS)

    enabled: bool = table.getBoolean("Enabled", False)

    screen.fill((0, 0, 0))
    events: list[pg.Event] = pg.event.get()
    for event in events:
        if event.type == pg.QUIT:
            pg.quit()
        elif event.type == pg.VIDEORESIZE:
            size = screen.get_size()
            field_render = pg.transform.scale_by(field.copy(), size[0] / field.get_width())
            pg.display.set_mode((screen.get_width(), field_render.get_height()), pg.RESIZABLE|pg.SRCALPHA)

    manager.update(dtime)

    match state:
        case State.REAL:
            # robotPose: list[int, int] = outputs.get
            print("Hello")
        case State.SIM:
            robotPose: list[int, int] = [0, 0]

    screen.blit(field_render, (0, 0))
    manager.draw_ui(screen)
    pg.display.flip()

    