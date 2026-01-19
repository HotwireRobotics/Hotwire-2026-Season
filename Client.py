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

size: tuple = (800, 600)

pg.display.set_caption(f"Firelight - {SERVER}")
pg.display.set_icon(pg.image.load(os.path.join(deploy, "logo.png")))
manager: pgui.UIManager = pgui.UIManager(size)
clock: pg.Clock = pg.Clock()

screen: pg.Surface = pg.display.set_mode(size, pg.RESIZABLE|pg.SRCALPHA)

label = pgui.elements.UILabel(
    relative_rect=pg.Rect(5, 5, 50, 50),
    text="Hello",
    manager=manager
)


while RUNNING:
    clock.tick(FPS)

    enabled: bool = table.getBoolean("Enabled", False)

    screen.fill((255, 255, 255))
    events: list[pg.Event] = pg.event.get()
    for event in events:
        if event.type == pg.QUIT:
            pg.quit()


    manager.draw_ui(screen)
    pg.display.flip()

    