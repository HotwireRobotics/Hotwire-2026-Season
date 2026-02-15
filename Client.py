from enum import Enum
import time, pygame as pg
import pygame_gui as pgui
import networktables as nt, io
import os, sys, math, requests
from math import prod
from networktables import NetworkTables as NT, NetworkTable

base: str = os.path.dirname(os.path.abspath(__file__))
deploy: str = os.path.join(base, "src", "main", "deploy")

SERVER: str = "10.29.90.2"
FPS: int = 60
RUNNING: bool = True
NT.initialize(server=SERVER)

table: NetworkTable =       NT.getTable("AdvantageKit")
driverStation =      table.getSubTable("DriverStation")
dashboard: NetworkTable = NT.getTable("SmartDashboard")
outputs =              table.getSubTable("RealOutputs")
camera: NetworkTable =   NT.getTable("CameraPublisher")
limelight =         camera.getSubTable("limelight-one")

pg.init()
pg.mixer.init()

sound: pg.Sound = pg.mixer.music.load(os.path.join(base, "assets", "rebuilt.mp3"))

field: pg.Surface = pg.image.load(os.path.join(base, "assets", "field.png"))
width: float = 0.7112
bumper: float = 0.2
field_render = field.copy()
size: tuple = field.get_size()

scale: float = size[1] / 9.18 # px/m
robot: pg.Surface = pg.Surface((width * scale, width * scale))
robot.fill((255, 0, 0))
pg.draw.rect(robot, (25, 25, 25), pg.Rect((scale * bumper), 
                                          (scale * bumper), 
                              width - 2 * (scale * bumper), 
                              width - 2 * (scale * bumper)), 2)
robot_render = robot.copy()

pg.display.set_caption(f"Firelight - {SERVER}")
pg.display.set_icon(pg.image.load(os.path.join(deploy, "logo.png")))
manager: pgui.UIManager = pgui.UIManager(size)
clock: pg.Clock = pg.Clock()

screen: pg.Surface = pg.display.set_mode(size, pg.RESIZABLE|pg.SRCALPHA)

class State(Enum):
    REAL: int = None
    SIM:  int = None
state: State = State.REAL

origin = [0.5537142857142857] * 2

def sarr(*arrays: list, integer: bool = False) -> list:
    return list(sum((round(array[i]) if integer else array[i]) for array in arrays) for i in range(len(arrays[0])))

def marr(*arrays: list, integer: bool = False) -> list:
    return list(prod((round(array[i]) if integer else array[i]) for array in arrays) for i in range(len(arrays[0])))

def px(real: list[int, int], integer: bool = False) -> list:
    n: float = marr(real, [scale * f] * 2)
    return list((round(i) for i in n) if integer else n)
bytes_data = b""

f: float = 1
pg.mixer.music.play()
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
            f = size[0] / field.get_width()
            field_render = pg.transform.scale_by(field.copy(), f)
            pg.display.set_mode((screen.get_width(), field_render.get_height()), pg.RESIZABLE|pg.SRCALPHA)
            robot_render: pg.Surface = pg.transform.scale_by(robot.copy(), f)

    manager.update(dtime)
    screen.blit(field_render, (0, 0))

    if (state == State.REAL):
        robotPose: list[int, int] = dashboard.getNumberArray("robot-pose", [0, 0])
        source: str = "http://limelight-one.local:5800/"
        stream = requests.get(source, stream=True)
    elif (state == State.SIM):
        robotPose: list[int, int] = [0, 0]
    
    est = sarr(robotPose, [-width / 2] * 2, origin)
    # print("Scale: ", scale, "F: ", f, "Pose: ", est, "Target: ", px(est, integer=True))
    screen.blit(robot, px(est, integer=True))
    manager.draw_ui(screen)
    pg.display.flip()