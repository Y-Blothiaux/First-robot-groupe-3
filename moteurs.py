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

def stop():
    dxl_io.set_moving_speed({1: 0, 2: 0})



def rotate_and_move(HorizontalOrientation):
    # Vitesse constante pour avancer
    baseSpeed = int(150 - (abs(HorizontalOrientation) * 0.5))
    
    coef = 2.2

    # Modèle vitesse de base + differenciel
    s_left = baseSpeed - (HorizontalOrientation * coef)
    s_right = baseSpeed + (HorizontalOrientation * coef)

    maxSpeed = 250
    s_left = max(-maxSpeed, min(maxSpeed, s_left))
    s_right = max(-maxSpeed, min(maxSpeed, s_right))

    dxl_io.set_moving_speed({1: s_right, 2: -s_left})

""" def go_to_xya(target_x, target_y, target_theta, dt):

    global x_inUse, y_inUse, theta_inUse, objectif_atteint

    diff_dist_max = 0.05  # 5 cm
    diff_angle_max = 0.03 # ~1.7 degré
    
    # 1. Calcul de l'écart avec la cible
    dx = target_x - x_inUse
    dy = target_y - y_inUse
    distance = math.sqrt(dx**2 + dy**2)
    
    #TODO faire une condition alternation si > a un angle par exemple 45%  du millieux on avance puis on tourne sinon on fais l'inverse
    if distance > diff_dist_max:
        # Phase d'avancement
        angle_cible = math.atan2(dy, dx)
        erreur_angle = normaliser_angle(angle_cible - theta_inUse)
        
        x_dot = SpeedLinear * distance * math.cos(erreur_angle)
        theta_dot = SpeedAngularLinear * erreur_angle
        objectif_atteint = False
    else:
        # Phase de rotation finale
        erreur_orientation = normaliser_angle(target_theta - theta_inUse)
        
        if abs(erreur_orientation) < diff_angle_max:
            # Cible atteinte !
            x_dot = 0.0
            theta_dot = 0.0
            objectif_atteint = True
        else:
            x_dot = 0.0
            theta_dot = SpeedAngular * erreur_orientation
            objectif_atteint = False

    # on transforme les données en vitesses réelle des roue pour reproduire le mouvement voulu
    v_left_deg, v_right_deg = inverse_kinematics(x_dot, theta_dot)
    
    
    # instruction aux moteurs
    dxl_io.set_moving_speed({1: v_left_deg, 2: -v_right_deg})
    
    # Mise à jour de l'odométrie 
    # en estimant la vitesse réelle du robot
    x_dot_reel, theta_dot_reel = direct_kinematics(v_left_deg, v_right_deg)
    x_inUse, y_inUse, theta_inUse = tick_odom(x_inUse, y_inUse, theta_inUse, x_dot_reel, theta_dot_reel, dt) """
    

direct= direct_kinematics(720,360)
angl_roues= inverse_kinematics(direct[0],direct[1])

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])

if __name__ == "__main__":

    dxl_io.set_moving_speed({1: -angl_roues[0], 2: angl_roues[1]}) # Degrés/s
    time.sleep(2)
    dxl_io.set_moving_speed({1: 0, 2: 0}) # Degrés/s