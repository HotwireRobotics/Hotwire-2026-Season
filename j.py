import pygame
import sys
import os, time


def main():
    # Initialize Pygame modules.
    pygame.init()
    pygame.mixer.init()
    pygame.joystick.init()

    BLIP = pygame.Sound("assets/blip.wav")
    SHOT = pygame.Sound("assets/shot.wav")
    SIXR = pygame.Sound("assets/6-7.mp3")
    GRUN = pygame.Sound("assets/grun.mp3")
    ELIT = pygame.Sound("assets/elit.mp3")
    ACES = pygame.Sound("assets/aces.mp3")
    QUAK = pygame.Sound("assets/quak.mp3")
    FAHH = pygame.Sound("assets/fahh.mp3")

    # Detect Xbox controller
    if pygame.joystick.get_count() == 0:
        print("No controller detected. Please connect an Xbox controller.")
        sys.exit(1)

    driver = pygame.joystick.Joystick(1)
    print(f"Controller detected: Drive")
    operator = pygame.joystick.Joystick(0)
    print(f"Controller detected: Operator")

    try:
        while True:
            for event in pygame.event.get():
                if (event.type == pygame.JOYHATMOTION):
                    if event.instance_id == 0:
                        SIXR.play()
                    else:
                        QUAK.play()
                if (event.type == pygame.JOYBUTTONDOWN):
                    if event.instance_id == 0:
                        if event.button == 5:
                            SHOT.play()
                        elif event.button == 4:
                            FAHH.play()
                        else:
                            GRUN.play()
                    else:
                        if event.button == 0:
                            ACES.play()
                        else:
                            ELIT.play()
                elif event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit(0)
    except KeyboardInterrupt:
        print("\nExiting...")
        pygame.quit()
        sys.exit(0)

if __name__ == "__main__":
    main()
