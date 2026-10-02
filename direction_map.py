import numpy as np
import cv2
import math
import time
from moteurs import rotate_and_move, stop
from goto import dxl_io, direct_kinematics, tick_odom
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

x = 0.0
y = 0.0
theta = 0.0
x_list = []
y_list = []
plot_update_counter = 0 # Pour mettre à jour le plot de manière synchrone avec la boucle (toutes les 5 itérations)
last = time.time()

def odometry(period=0.02):
    global x, y, theta, x_list, y_list, plot_update_counter, last
    # dt réel mesuré
    now = time.time()
    dt = now - last
    last = now

    # On lit la vitesse des roues poussées
    speeds = dxl_io.get_present_speed([1, 2]) # en deg/s
    v_left = speeds[0]
    v_right = -speeds[1]
    x_dot, theta_dot = direct_kinematics(v_left, v_right)
    y, x, theta = tick_odom(y, x, theta, x_dot, theta_dot, dt)

    x_cm = -x * 100
    y_cm = y * 100

    x_list.append(x_cm)
    y_list.append(y_cm)

    print(f"\rX: {x_cm:7.1f} cm | Y: {y_cm:7.1f} cm | Cap: {math.degrees(theta):7.1f}°", end="")

    line.set_data(x_list, y_list)
    robot.set_data(x_list[-1:], y_list[-1:])
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.001) # Pause nécessaire à la MAJ
    return x, y, theta, x_list, y_list, plot_update_counter, last

# Declaration HSV des couleurs
lower_blue = np.array([100, 110, 50])
upper_blue = np.array([135, 255, 255])
lower_green = np.array([65, 72, 57])
upper_green = np.array([98, 255, 255])
lower_yellow = np.array([15, 40, 40])
upper_yellow = np.array([45, 255, 255])
lower_red1 = np.array([0, 40, 40])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 40, 40])
upper_red2 = np.array([179, 255, 255])

numeroCouleur = 0       # Identifiant de la couleur actuelle
attenteVert = 90        # Delai d'attente en frame avant de pouvoir detecter le vert, 3 seconde par defaut (30fps)
compteurVert = 0        # Nombre de frame d'affilé sur lequel on a capté du vert
lower = lower_yellow    # Premiere couleur du cycle (yellow)
upper = upper_yellow

# Prends une frame et renvoi un entier entre -100 (il faut tourner a gauche),
# 0 (tout droit) et 100 (droite) (gauche droite peut etre inversé)
def prediction(img):
    global numeroCouleur, lower, upper, attenteVert, compteurVert
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)      # On passe l'image du champ rgb vers hsv pour detecter les couleurs plus facilement
    attenteVert-=1                                      # On met a jour l'attente du vert pour cette frame

    if attenteVert<=0:
        # Recherche de zone verte (masquage et contour)
        mask = cv2.inRange(hsv_img, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # On verifie qu'on a capté un contour d'un objet vert 
        if contours:
            largestContour = max(contours, key=cv2.contourArea)
            moment = cv2.moments(largestContour)
            if (moment["m00"] > 800):
                print(compteurVert)
                compteurVert+=1
                # Si on a detecté X frame d'affilé contenant du vert, on change la couleur
                if compteurVert >= 2:
                    attenteVert = 180   # On initialise le delai avant redetectionn de vert possible a 6sec
                    compteurVert = 0
                    numeroCouleur += 1
                    match numeroCouleur:
                        case 0:
                            lower = lower_yellow
                            upper = upper_yellow
                            print("yellow")
                        case 1:
                            lower = lower_blue
                            upper = upper_blue
                            print("bleue")
                        case 2:
                            # rouge géré plus bas
                            print("rouge")
                        case 3:
                            rotate_and_move(0,1)
                            time.sleep(0.25)
                            stop()
                            exit()
                    print(lower)
            else :
                compteurVert=0
        else :
            compteurVert=0

    # On creer un masque de l'image par un filtre de couleur, en mettant en blanc 
    # les pixels de la range de couleur cherché (et les autres en noir)
    # Le rouge est divisé en 2 alors on doit fusionner les 2 etallonages
    if numeroCouleur == 2:
        mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
    else :
        mask = cv2.inRange(hsv_img, lower, upper)

    # cv2.imshow("prediciton masque debug", mask)

    # On extrait les contours de l'image pour la couleur choisi
    # contours : liste python des contours detecté, sous forme de tableau numpy 
    # de dimension (N, 1, 2), avec N le nombre de points reliés qui forme ce countour.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # On recupere le plus grand contour (la bande coloré)
        largestContour = max(contours, key=cv2.contourArea)
        moment = cv2.moments(largestContour)
        #cv2.imshow("prediciton contour debug", cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3))

        # moment["m00"] represente la surface, le nombre de pixel de ce contour
        if (moment["m00"] > 300):
            cx = int(moment["m10"]/moment["m00"])       # On calcul la position du centre du contour (sur l'axe X)
            centreImage = (img.shape[1]/2)              # On recupere le centre de l'image sur l'axe X
            erreur = cx - centreImage
            direction = int((erreur/centreImage)*100)   # On le converti sur l'echelle "-100" à "100"
            direction = max(-100,min(100,direction))    # On force la direction dans cet echelle

            #cv2.putText(img, str(direction), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            #cv2.imshow("prediciton debug", cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3))
            return direction
    # Si aucune ligne n'est trouvé, on va tout droit
    #cv2.imshow("prediciton debug", img)
    return 0


if __name__ == "__main__":
    frameCounter=0
    # Choix de la camera (1 si pc portable avec webcam intégré, 0 si rasb)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    fig, ax = plt.subplots(figsize=(8, 8))
    line, = ax.plot(x_list, y_list, 'b-', label="Trajectoire")
    robot, = ax.plot(x_list[-1:], y_list[-1:], 'ro', label="Robot")

    try:
        # Boucle infini de capture de la camera
        while True:
            # On recupere une frame de la camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Can't receive frame.")
                break

            frame = cv2.resize(frame, (360, 240))       # On force une resolution plus legere pour le traitement
            frame = frame[100:360, :]                   # On coupe pour garder le bas de l'image
            direction = prediction(frame)               # On recupere une predication de la direction a prendre
            rotate_and_move(direction,2)
            frameCounter+=1
            if frameCounter>=30:
                odometry()
                frameCounter=0

            # Appuyer sur q pour quitter la boucle (a virer)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                plt.show()
                stop()
                break
    finally:
        stop()
        cap.release()
        cv2.destroyAllWindows()

        ax.set_title("Cartographie par Odométrie Temps Réel")
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.grid(True)
        ax.legend()
        ax.axis('equal') # Échelle 1:1 pour ne pas déformer la trajectoire

        plt.savefig("trajectoire_parcours.png")
        print("Graphique sauvegardé sous 'trajectoire_parcours.png'.")
