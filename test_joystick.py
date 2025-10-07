import pygame
pygame.init()
pygame.joystick.init()
import time
joystick = pygame.joystick.Joystick(0)
joystick.init()
#list all axis in joystick
axes=joystick.get_numaxes()
# where hand control comes
# axis 0 roll
# axis 1 pitch
# axis 2 thrust
# axis 3 yaw
# axis 4 too cheem
# axis 5 middle
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    for axis in range(axes):
        axis_value=joystick.get_axis(axis)
        print("Axis {} value: {:>6.3f}".format(axis, axis_value))
    time.sleep(0.3)
    lowest = 0.9
    highest = -1
    range_js = -(highest - lowest)
    range_motor = 65535
    rate = range_motor / range_js

    a0 = joystick.get_axis(0)  # x axis right hand <- ->
    a1 = joystick.get_axis(1)  # y axis right hand up down
    a2 = joystick.get_axis(2)  # thrust
    a3 = joystick.get_axis(3)

    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)