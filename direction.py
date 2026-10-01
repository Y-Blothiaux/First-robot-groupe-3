import numpy as np
import matplotlib.pyplot as plt
import cv2
import time

from moteurs import rotate_and_move, stop

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

# Identifiant de la couleur actuelle
numeroCouleur = 0

# Delai d'attente en frame avant de pouvoir detecter le vert, 3 seconde par defaut (30fps)
attenteVert = 90

# Nombre de frame d'affilé sur lequel on a capté du vert
compteurVert = 0

# Premiere couleur du cycle (bleue)
lower = lower_yellow
upper = upper_yellow

# Fonction de prediction
# Prends une frame et renvoi un entier entre -100 (il faut tourner a gauche),
# 0 (tout droit) et 100 (droite) (gauche droite peut etre inversé)
def prediction(img):
    # On recupere les variables globales pour cette fonction
    global numeroCouleur, lower, upper, attenteVert, compteurVert

    # On passe l'image du champ rgb vers hsv pour detecter les couleurs plus facilement
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # On met a jour l'attente du vert pour cette frame
    attenteVert-=1

    # On verifie que la duree d'attente avant redection possible est terminé
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

                # On incremente le compteur de zone verte détecté d'affilé 
                compteurVert+=1

                # Si on a detecté X frame d'affilé contenant du vert, on change la couleur
                if compteurVert>=2:

                    # On initialise le delai avant redetectionn de vert possible a 6sec
                    attenteVert=180

                    compteurVert = 0

                    # On incremente l'identifiant de couleur actuelle de 1 avec modulo pour cycle
                    numeroCouleur=numeroCouleur+1

                    # On regarde la nouvelle couleur par l'identifiant pour choisir la bonne couleur hsv
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
                            sleep(1)
                            stop()
                            return 
                    print(lower)

            else :
                compteurVert=0
        else :
            # Si on pas detecté de zone verte d'affilé, on reset le compteur
            compteurVert=0


    # Recherche de ligne coloré

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

        #cv2.imshow("prediciton contour debug", cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3))

        # On recupere les infos physiques sur ce contour
        moment = cv2.moments(largestContour)

        # moment["m00"] represente la surface, le nombre de pixel de ce contour
        if (moment["m00"] > 300):

            # On calcul la position du centre du contour (sur l'axe X) 
            cx = int(moment["m10"]/moment["m00"])
            
            # On recupere le centre de l'image sur l'axe X
            centreImage = (img.shape[1]/2)
            
            # On calcul l'ecart du centre du contour par rapport au centre
            erreur = cx - centreImage
            
            # On le converti sur l'echelle "-100" à "100"
            direction = int((erreur/centreImage)*100)
            
            # On force la direction dans cet echelle
            direction = max(-100,min(100,direction))

            #cv2.putText(img, str(direction), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            #cv2.imshow("prediciton debug", cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3))

            return direction

    # Si aucune ligne n'est trouvé, on va tout droit
    #cv2.imshow("prediciton debug", img)
    return 0


if __name__ == "__main__":

    # On choisit la camera (1 si pc portable avec webcam intégré, 0 si rasb)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()

    # Boucle infini de capture de la camera
    while True:

        # On recupere une frame de la camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame.")
            break

        # cv2.imshow('Main debug', extraction(frame))

        # On force une resolution plus legere pour le traitement
        frame = cv2.resize(frame, (360, 240))

        # On coupe pour garder le bas de la frame
        frame = frame[100:360, :]

        # On recupere une predication de la direction a prendre
        direction = prediction(frame)

        # On appelle la fonction moteurs avec la prediction obtenu
        rotate_and_move(direction, numeroCouleur)

        # Appuyer sur q pour quitter la boucle (a virer)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop()
            break


    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()
