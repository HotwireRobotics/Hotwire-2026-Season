# Test.py
import sys
import os

def clear(*s: str):
    """Clear console."""
    os.system('cls' if os.name == 'nt' else 'clear')

    # Replace with strung arguments.
    if (s): print(*s)

# Define conversion constant from inches to meters.
constant: float = 0.0254

clear("------------------")

# Collect inputs.
right  : float = float(eval(input(  "right (in): ")))
up     : float = float(eval(input(     "up (in): ")))
forward: float = float(eval(input("forward (in): ")))

print("------------------")

# Print resulting.
print(  "right: ", constant * right, "m")
print(     "up: ", constant * up, "m")
print("forward: ", constant * forward, "m")

print("------------------")

# GAMMA
# 19.375in UP
# 13.5 - 2.5in RIGHT
# 13.5 - (3 + 5/8)in FORWARD

# ALPHA
# 17.25in UP
# 13.5 - 3.375in RIGHT
# 13.5 - .5

# DISTANCE (in)   SPEED (RPM)
#    118.5     |     2500
#     75.0     |     2050
#    135.0     |     2800
#    102.0     |     2340
#    156.0     |     3120