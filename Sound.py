# Sound.py
import pygame as pg, time
from pygame.joystick import JoystickType

# Initialize the joystick module and the mixer module.
pg.init()
pg.joystick.init(); pg.mixer.init()

# Create a list of Joystick objects for each connected joystick.
joysticks: list[JoystickType] = [pg.joystick.Joystick(i) for i in range(pg.joystick.get_count())]

# Iterate through the joysticks and print their names.
for joy in joysticks:
    print(f"Initialized: {joy.get_name()}")

sound: list[pg.mixer.Sound] = [
    pg.mixer.Sound("assets/rebuilt.mp3")
]

# Initialize screen.
pg.display.set_caption("Sounder")
screen: pg.Surface = pg.display.set_mode((200, 160), pg.RESIZABLE|pg.SRCALPHA)

# Loop function.
while True:
    # Handle events.
    # events: list[pg.Event] = pg.event.get()
    # for event in events:
    #     if event.type == pg.QUIT: pg.quit(); exit()
    #     if event.type == pg.JOYBUTTONDOWN:
    #         if event.button >= len(sound):
    #             sound[0].play()
    #         else:
    #             sound[event.button].play()

    # for joy in joysticks:
    #     joy.rumble(0.0, 1.0, 100)
    #     print(joy.)    
    #     if joy.get_button(1):
    sound[0].play()
