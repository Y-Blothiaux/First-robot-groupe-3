import math
import pypot.dynamixel
import time

R = 0.025  # rayon de la roue = 2.5 cm
L = 0.15  # distance entre les roues = 15 cm

def inverse_kinematics(x_dot, theta_dot):

    v_left_rad = (x_dot - (theta_dot * L / 2.0)) / R
    v_right_rad = (x_dot + (theta_dot * L / 2.0)) / R
    
    v_left_deg = math.degrees(v_left_rad)
    v_right_deg = math.degrees(v_right_rad)
    
    return v_left_deg, v_right_deg


def direct_kinematics(v_left, v_right):

    v_left_rad = math.radians(v_left)
    v_right_rad = math.radians(v_right)
    
    x_dot = R * (v_left_rad + v_right_rad) / 2.0
    theta_dot = R * (v_right_rad - v_left_rad) / L
    
    return x_dot, theta_dot

def odom(x_dot, theta_dot, dt):
    delta_theta = theta_dot * dt

    if abs(theta_dot) > 1e-3:
        r = x_dot / theta_dot 
        delta_x = r * math.sin(delta_theta)
        delta_y = r * (1.0 - math.cos(delta_theta))
    else:
        delta_x = x_dot * dt
        delta_y = 0.0

    return delta_x, delta_y, delta_theta   


def go_to_xya(x,y,theta):
    diff_dist_max= 0.05 #
    diff_angle_max=0.03 
    
direct= direct_kinematics(720,360)
angl_roues= inverse_kinematics(direct[0],direct[1])

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])
dxl_io.set_moving_speed({1: angl_roues[0], 2: -angl_roues[1]}) # Degrees / s
time.sleep(2)
dxl_io.set_moving_speed({1: 0, 2: 0}) # Degrees / s
