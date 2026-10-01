import numpy as np
import matplotlib.pyplot as plt
import cv2

#from moteurs import rotate_and_move, stop


# BLEU
lower_blue = np.array([100, 110, 50])
upper_blue = np.array([135, 255, 255])

# VERT
lower_green = np.array([65, 72, 57])
upper_green = np.array([101, 255, 255])

# JAUNE
lower_yellow = np.array([15, 40, 40])
upper_yellow = np.array([45, 255, 255])

# ROUGE (Le rouge est coupé en deux sur l'échelle OpenCV)
lower_red1 = np.array([0, 40, 40])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 40, 40])
upper_red2 = np.array([179, 255, 255])

numeroCouleur = 0
attente = 0

lower = lower_green
upper = upper_green

def prediction(img):
    global numeroCouleur, lower, upper, attente

    # On passe l'image du champ rgb vers hsv pour detecter les couleurs plus facilement
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    attente-=1

    # Recherche de zone verte
    mask = cv2.inRange(hsv_img, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    attente = 1 
    if contours and attente<=0:
        largestContour = max(contours, key=cv2.contourArea)
        moment = cv2.moments(largestContour)
        if (moment["m00"] > 300):
            attente=180
            numeroCouleur=(numeroCouleur+1)%3
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
                    lower = lower_red1
                    upper = upper_red1
                    print("rouge")
            print(lower)

    # Recherche de ligne coloré

    # On creer un masque de l'image par un filtre de couleur, en mettant en blanc 
    # les pixels de la range de couleur cherché (et les autres en noir)
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

            cv2.putText(img, str(direction), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow("prediciton debug", cv2.drawContours(img, largestContour, -1, (0, 255, 0), 3))

            return direction

    # Si aucune ligne n'est trouvé, on va tout droit
    cv2.imshow("prediciton debug", img)
    return 0


if __name__ == "__main__":

    # On choisit la camera (1 si pc portable, 0 sinon)
    cap = cv2.VideoCapture(1)
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

        #rotate_and_move(direction)

        # Appuyer sur q pour quitter la boucle (a virer)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop()
            break


    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()
