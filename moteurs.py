import math
import pypot.dynamixel
import time

R = 0.025  # Rayon de la roue = 2.5 cm
L = 0.15  # Distance entre les roues = 15 cm

# Variables de position du robot dans le monde
# Modifiées à chaque cycle de 20ms
x_inUse = 0.0
y_inUse = 0.0
theta_inUse = 0.0

# Config des vitesses
SpeedLinear = 2.0
SpeedAngularLinear = 3.0
SpeedAngular = 2.0




def stop():
    dxl_io.set_moving_speed({1: 0, 2: 0})



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
