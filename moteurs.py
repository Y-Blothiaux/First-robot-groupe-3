import math
import pypot.dynamixel
import time

# Methode auxiliaire pour stopper proprement les moteurs
def stop():
    dxl_io.disable_torque([1,2])

# On setup les moteurs avec les bon port 
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')
dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])

# Fait avancer et tourner le robot en fonction de 
# - HorizontalOrientation, un valeur echalonné entre -100 (il faut tourner a fond a droite);
# 0 (tout droit) et 100 (a fond a gauche)
# - numeroCouleur represente la couleur actuelle de la ligne et permet d'accelerer
# en fonction de sa difficulté
def rotate_and_move(HorizontalOrientation, numeroCouleur):

    match numeroCouleur:
        # jaune, on va vite
        case 0:
            # La vitesse diminue si le coefficient HorizontalOrientation est haut
            baseSpeed = int(800 - (abs(HorizontalOrientation) * 0.5))
        # bleue, vitesse moyenne
        case 1:
            baseSpeed = int(600 - (abs(HorizontalOrientation) * 0.5))
        # rouge, on ralenti beaucoup
        case 2:
            baseSpeed = int(350 - (abs(HorizontalOrientation) * 4))

    # Sécurité pour empêcher une vitesse de base négative
    #baseSpeed = max(50,baseSpeed)

    # Amplifie l'impacte de HorizontalOrientation dans la rotation des roues
    coef = 2.9
    # Calcul individuel pour chaque roue en fonction des donnée du capteur
    s_left = baseSpeed - (HorizontalOrientation * coef)
    s_right = baseSpeed + (HorizontalOrientation * coef)

    # On filtre la vitesse pour qu'elle reste dans des limites acceptables par sécurité
    maxSpeed = 500
    s_left = int(max(-maxSpeed, min(maxSpeed, s_left)))
    s_right = int(max(-maxSpeed, min(maxSpeed, s_right)))

    # On fini par ecrire les vitesse obtenu dans les moteurs
    dxl_io.set_moving_speed({1: s_right, 2: -s_left})
