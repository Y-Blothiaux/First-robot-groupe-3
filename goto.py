import math
import pypot.dynamixel
import time


R = 0.025  # rayon de la roue = 2.5 cm
L = 0.15  # distance entre les roues = 15 cm


# Variables de position du robot dans le monde
#  modifiées à chaque cycle de 20ms
x_inUse = 0.0
y_inUse = 0.0
theta_inUse = 0.0

x_dot_inUse = 0.0
y__dot_inUse = 0.0
theta_dot_inUse = 0.0
dt_inUse= 0.02
# Config des vitesses 
speedLinear = 2.0
speedAngularLinear = 3.0
speedAngular = 2.0


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

def odom2D(x_dot, theta_dot, dt):
     delta_theta = theta_dot * dt
     if abs(theta_dot) > 1e-3:
        ray_traj = x_dot / theta_dot 
        delta_x = ray_traj * math.sin(delta_theta)
        delta_y = ray_traj * (1.0 - math.cos(delta_theta))
     else:
        delta_x = x_dot * dt
        delta_y = 0.0
     return delta_x, delta_y, delta_theta   

def tick_odom(x, y, theta, x_dot, theta_dot, dt):
    i,j,k= odom2D(x_dot,theta_dot,dt)
    #changement de repère
    new_x = x + (i * math.cos(theta) - j * math.sin(theta))
    new_y = y + (i * math.sin(theta) + j * math.cos(theta))
    new_theta = normaliser_angle(theta + k)
    return new_x, new_y, new_theta 

def normaliser_angle(angle):
    # Ramène un angle en radians dans l'intervalle [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


def go_to_xya(target_x,target_y,target_theta,dt):
    global x_inUse,y_inUse,theta_inUse
    target_diff_dist_max = 0.05  # 5 cm
    target_diff_angle_max = 0.03
    tolerance_cap_marche = 0.1   # Tolérance de cap pour autoriser l'avancement (rad, env. 5°)

    vitesse_marche = 0.15
    vitesse_rotation = 1.0
    while True:
        speeds = dxl_io.get_present_speed([1, 2])
        v_left_inUse = speeds[0]
        v_right_inUse = -speeds[1]
        x_dot_inUse,theta_dot_inUse=direct_kinematics(v_left_inUse,v_right_inUse)
        x_inUse, y_inUse, theta_inUse = tick_odom(x_inUse, y_inUse, theta_inUse, x_dot_inUse, theta_dot_inUse, dt)
        #distance restante avec pythagore
        dx = target_x - x_inUse
        dy = target_y - y_inUse
        distance_restante = math.sqrt(dx**2 + dy**2)

        if distance_restante > target_diff_dist_max:
            #
            angle_vers_point = math.atan2(dy, dx)
            erreur_cap = normaliser_angle(angle_vers_point - theta_inUse)
            
            if abs(erreur_cap) > tolerance_cap_marche:
                # alignement avec la cible
                x_dot_target = 0.0
                if erreur_cap > 0:
                    theta_dot_target = vitesse_rotation 
                else: theta_dot_target = -vitesse_rotation
            else:
                
                x_dot_target = vitesse_marche
                theta_dot_target = 0.0
                
        else:
           # on pivote vers l'angle final
            error_angle_final = normaliser_angle(target_theta - theta_inUse)
            
            if abs(error_angle_final) > target_diff_angle_max:
                # On tourne sur place pour corriger l'angle final
                x_dot_target = 0.0
                if error_angle_final > 0:
                      theta_dot_target = 0.1
                else: theta_dot_target = -0.1 
            else:
                dxl_io.set_moving_speed({1: 0, 2: 0})
                break

        
        v_left_deg, v_right_deg = inverse_kinematics(x_dot_target, theta_dot_target)
        dxl_io.set_moving_speed({1: v_left_deg, 2: -v_right_deg})
        
        time.sleep(dt)



        
        
        
direct= direct_kinematics(720,360)
angl_roues= inverse_kinematics(direct[0],direct[1])

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])

if __name__ == "__main__":
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit('No port')

    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])
    
    # S'assurer que le robot est à l'arrêt avant de commencer
    dxl_io.set_moving_speed({1: 0, 2: 0})
    time.sleep(1)

    # x = 0.5m, y = 0.0m, angle final = 0°
    go_to_xya(0.5, 0.0, 0.0, dt_inUse)
    
    time.sleep(2) # Pause de 2 secondes
    
    # x = 0.5m, y = 0.5m, angle final = 90° (converti en radians)
    go_to_xya(0.5, 0.5, math.radians(90), dt_inUse)

    # x = 0.0m, y = 0.0m, angle final = 180°
    go_to_xya(0.0, 0.0, math.radians(180), dt_inUse)

    # Arrêt de sécurité
    dxl_io.set_moving_speed({1: 0, 2: 0})