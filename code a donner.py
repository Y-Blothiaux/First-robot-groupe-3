import math
import pypot.dynamixel
import time

# Constantes physiques 
RayonRoue = 0.025  # Rayon de la roue = 2.5 cm
DistanceRoue = 0.15   # Distance entre les roues = 15 cm

# Gains de contrôle globaux
SpeedLinear = 2.0
SpeedAngularLinear = 3.0
SpeedAngular = 2.0


def inverse_kinematics(x_dot, theta_dot):
    v_left_rad = (x_dot - (theta_dot * DistanceRoue / 2.0)) / RayonRoue
    v_right_rad = (x_dot + (theta_dot * DistanceRoue / 2.0)) / RayonRoue
    
    v_left_deg = math.degrees(v_left_rad)
    v_right_deg = math.degrees(v_right_rad)
    
    return v_left_deg, v_right_deg

def direct_kinematics(v_left, v_right):
    v_left_rad = math.radians(v_left)
    v_right_rad = math.radians(v_right)
    
    x_dot = RayonRoue * (v_left_rad + v_right_rad) / 2.0
    theta_dot = RayonRoue * (v_right_rad - v_left_rad) / DistanceRoue
    
    return x_dot, theta_dot

def normaliser_angle(angle):
    # Ramène un angle en radians dans l'intervalle [-pi, pi]
    return math.atan2(math.sin(angle), math.cos(angle))

def go_to_xya(target_x, target_y, target_theta, dt):





    def go_to_xya(target_x, target_y, target_theta, dt):

    x_inUse, y_inUse, theta_inUse, objectif_atteint

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
    x_inUse, y_inUse, theta_inUse = tick_odom(x_inUse, y_inUse, theta_inUse, x_dot_reel, theta_dot_reel, dt) 

 def tick_odom(x, y, theta, x_dot, theta_dot, dt):
    # Calcule la variation dans le repère robot
    delta_theta = theta_dot * dt
    if abs(theta_dot) > 1e-3:
        r = x_dot / theta_dot
        delta_x_robot = r * math.sin(delta_theta)
        delta_y_robot = r * (1.0 - math.cos(delta_theta))
    else:
        delta_x_robot = x_dot * dt
        delta_y_robot = 0.0

    #(changement de repère)
    new_x = x + (delta_x_robot * math.cos(theta) - delta_y_robot * math.sin(theta))
    new_y = y + (delta_x_robot * math.sin(theta) + delta_y_robot * math.cos(theta))
    new_theta = normaliser_angle(theta + delta_theta)
    
    return new_x, new_y, new_theta 

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

# Variables de position du robot dans le monde
#  modifiées à chaque cycle de 20ms
x_actuel = 0.0
y_actuel = 0.0
theta_actuel = 0.0

# Config des vitesses 
SpeedLinear = 2.0
SpeedAngularLinear = 3.0
SpeedAngular = 2.0





# =====================================================================
# EXEMPLE D'INTEGRATION DANS TA BOUCLE DE 20MS (CAMÉRA)
# =====================================================================

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])

# Initialisation de la position estimée du robot au départ (repère monde)
x_robot, y_robot, theta_robot = 0.0, 0.0, 0.0

# Ta cible finale souhaitée
target_x, target_y, target_theta = 0.5, 0.3, math.pi/2  # 50cm en X, 30cm en Y, orienté à 90°

print("Début de la navigation...")
t_precedent = time.time()
arrive = False

while not arrive:
    t_actuel = time.time()
    dt = t_actuel - t_precedent
    t_precedent = t_actuel
    
    # Éviter un dt nul au premier cycle
    if dt <= 0:
        dt = 0.020 
        
    # [OPTIONNEL] Si ta caméra te donne x_robot, y_robot, theta_robot à cet instant,
    # tu peux écraser les variables ici avec les vraies valeurs mesurées.
    
    # Calculer les vitesses requises pour ce cycle de 20ms
    v_l, v_r, arrive = go_to_xya(target_x, target_y, target_theta, x_robot, y_robot, theta_robot, dt)
    
    # Appliquer les commandes aux Dynamixels (attention au signe inversé selon le montage du moteur 2)
    dxl_io.set_moving_speed({1: v_l, 2: -v_r})
    
    # Si on n'a pas de caméra pour corriger, on met à jour la position par odométrie pure
    # On repasse par direct_kinematics pour estimer la vitesse réelle appliquée
    x_dot_reel, theta_dot_reel = direct_kinematics(v_l, v_r)
    x_robot, y_robot, theta_robot = tick_odom(x_robot, y_robot, theta_robot, x_dot_reel, theta_dot_reel, dt)
    
    # Contrôle de la fréquence de boucle (~20 ms)
    time.sleep(0.020)

# Arrêt final des moteurs
dxl_io.set_moving_speed({1: 0, 2: 0})
print("Le robot est arrivé à destination !")
