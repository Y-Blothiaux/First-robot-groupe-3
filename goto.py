import math
import pypot.dynamixel
import time


R = 0.025  # rayon de la roue = 2.5 cm
L = 0.146  # distance entre les roues = 14.6 cm


# Variables de position du robot dans le monde
# Modifiées à chaque cycle de 20ms
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

#initialisation
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


def inverse_kinematics(x_dot, theta_dot):
    # Convertit la vitesse de déplacement globale souhaitée (avance x_dot, rotation theta_dot)
    # en vitesse de rotation spécifique pour chaque roue.
    # L/2 est la distance du centre du robot jusqu'à la roue (bras de levier).
    v_left_rad = (x_dot - (theta_dot * L / 2.0)) / R
    v_right_rad = (x_dot + (theta_dot * L / 2.0)) / R
    # Les moteurs Dynamixel attendent des consignes en degrés/seconde,
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
     # Calcule le déplacement "local" du robot 
     delta_theta = theta_dot * dt
     # Si la vitesse angulaire est significative, le robot ne va pas tout droit : 
     # il décrit un arc de cercle.
     if abs(theta_dot) > 1e-3:
        # ray_traj correspond au rayon de courbure de cet arc de cercle.
        ray_traj = x_dot / theta_dot 
        delta_x = ray_traj * math.sin(delta_theta)
        # 1 - cos(theta) donne le glissement latéral inhérent à un déplacement en courbe.
        delta_y = ray_traj * (1.0 - math.cos(delta_theta))
     else:
        # Si la rotation est quasi nulle, on simplifie avec un déplacement droit pur
        # pour éviter une division par zéro (x_dot / 0).
        delta_x = x_dot * dt
        delta_y = 0.0
     return delta_x, delta_y, delta_theta   

def tick_odom(x, y, theta, x_dot, theta_dot, dt):
    i,j,k= odom2D(x_dot,theta_dot,dt)
    # Changement de repère : on passe du repère local du robot (i, j) 
    # au repère global du monde (x, y) en appliquant une matrice de rotation 2D.
    new_x = x + (i * math.cos(theta) - j * math.sin(theta))
    new_y = y + (i * math.sin(theta) + j * math.cos(theta))
    # On normalise le nouvel angle pour éviter qu'à force de tourner
    # les valeurs dépassent l'intervalle [-pi, pi].
    new_theta = normalise_angle(theta + k)
    return new_x, new_y, new_theta 

def normalise_angle(angle):
    # Ramène un angle en radians dans l'intervalle [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))


def go_to_xya(target_x,target_y,target_theta,dt):
    global x_inUse,y_inUse,theta_inUse
    target_diff_dist_max = 0.03  # 3 cm
    target_diff_angle_max = 0.03
    tolerated_cap_steps = 0.05   # Tolérance de cap pour autoriser l'avancement (rad, env. 5°)

    vitesse_marche = 0.15
    speed_rotation = 3.0
    while True:
        # On lit la vitesse réelle des moteurs (boucle fermée).
        # L'inversion de signe sur la roue droite (-speeds[1]) est nécessaire car 
        # les deux moteurs sont physiquement montés en miroir sur le châssis
        speeds = dxl_io.get_present_speed([1, 2])
        v_left_inUse = speeds[0]
        v_right_inUse = -speeds[1]
        
        x_dot_inUse,theta_dot_inUse=direct_kinematics(v_left_inUse,v_right_inUse)
        #on actualise la position
        x_inUse, y_inUse, theta_inUse = tick_odom(x_inUse, y_inUse, theta_inUse, x_dot_inUse, theta_dot_inUse, dt)
        # Distance restante avec pythagore
        dx = target_x - x_inUse
        dy = target_y - y_inUse
        distance_togo = math.sqrt(dx**2 + dy**2)

        if distance_togo > target_diff_dist_max:
            # Le robot est trop loin du point d'arrivée.
            # On calcule la ligne droite (angle_to_point) reliant le robot à sa cible.
            angle_to_point = math.atan2(dy, dx)
            error_cap = normalise_angle(angle_to_point - theta_inUse)
            # Logique de navigation (Machine à états) : 
            # On priorise la rotation. Si le robot n'est pas bien aligné
            # avec la cible (erreur > tolérance), il pivote sur place.
            tolerance_actuelle = tolerated_cap_steps
            if distance_togo < 0.15:
                tolerance_actuelle=0.25
            vitesse_avance = max(0.05,min(vitesse_marche,2*distance_togo))
            if abs(error_cap) > tolerance_actuelle:
                # Alignement avec la cible
                x_dot_target = 0.0
                theta_dot_target = max(-speed_rotation,min(speed_rotation, 2* error_cap))
            else:
                # S'il est bien aligné, alors et seulement alors, il avance en ligne droite.
                x_dot_target = vitesse_avance
                theta_dot_target = 0.0
                
        else:
           # Le robot a atteint les coordonnées (x, y). 
           # Il effectue maintenant sa rotation finale sur place pour atteindre target_theta.
            error_angle_final = normalise_angle(target_theta - theta_inUse)
            
            if abs(error_angle_final) > target_diff_angle_max:
                # On tourne sur place pour corriger l'angle final
                x_dot_target = 0.0
                theta_dot_target = max(-speed_rotation, min(speed_rotation, 2 * error_angle_final)) 
            else:
                dxl_io.set_moving_speed({1: 0, 2: 0})
                break

        
        v_left_deg, v_right_deg = inverse_kinematics(x_dot_target, theta_dot_target)
        dxl_io.set_moving_speed({1: v_left_deg, 2: -v_right_deg})
        
        time.sleep(dt)       



        
direct= direct_kinematics(720,360)
angl_roues= inverse_kinematics(direct[0],direct[1])

#tests

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

    time.sleep(2)
    # retour au départ
    go_to_xya(0.0, 0.0, math.radians(-135), dt_inUse)

    # Arrêt de sécurité
    dxl_io.set_moving_speed({1: 0, 2: 0})
