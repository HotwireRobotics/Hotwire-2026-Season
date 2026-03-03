import sys
import os

def clear():
    os.system('cls' if os.name == 'nt' else 'clear')

clear()
right  : float = float(input(  "right (in): "))
up     : float = float(input(     "up (in): "))
forward: float = float(input("forward (in): "))

constant: float = 0.0254
print("------------------")

print(  "right: ", constant * right, "m")
print(     "up: ", constant * up, "m")
print("forward: ", constant * forward, "m")