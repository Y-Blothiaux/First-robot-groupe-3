import math
import pypot.dynamixel
import time





def stop():
    dxl_io.set_moving_speed({1: 0, 2: 0})


ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


def rotate_and_move(HorizontalOrientation):
    # Vitesse constante pour avancer
    baseSpeed = int(400 - (abs(HorizontalOrientation) * 0.5))
    
    coef = 2.8

    # Modèle vitesse de base + differenciel
    s_left = baseSpeed - (HorizontalOrientation * coef)
    s_right = baseSpeed + (HorizontalOrientation * coef)

    maxSpeed = 700
    s_left = max(-maxSpeed, min(maxSpeed, s_left))
    s_right = max(-maxSpeed, min(maxSpeed, s_right))

    dxl_io.set_moving_speed({1: s_right, 2: -s_left})
