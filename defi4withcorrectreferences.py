import numpy as np
import cv2
import math
import time
import json

from moteurs import rotate_and_move, stop
from goto import dxl_io, direct_kinematics, tick_odom

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')


# ============================================================
# CORRECTION DES "NORDS" DES DIFFERENTS CIRCUITS
# ============================================================
#
# Le jaune sert de repere de reference.
#
# Exemples :
#   90  = tourner le circuit de 90° sens anti-horaire
#   -90 = tourner le circuit de 90° sens horaire
#
# IMPORTANT :
# ces valeurs ne changent NI la vitesse du robot,
# NI la forme du circuit,
# NI son echelle.
#
# Elles tournent uniquement les points cartographies.
#
# Pour commencer, laisser a 0.
# On ajustera ensuite uniquement BLEU et ROUGE.
# ============================================================

ROTATION_JAUNE_DEG = 0.0
ROTATION_BLEU_DEG = 0.0
ROTATION_ROUGE_DEG = 0.0

rotation_repere_deg = {
    0: ROTATION_JAUNE_DEG,
    1: ROTATION_BLEU_DEG,
    2: ROTATION_ROUGE_DEG
}


# ============================================================
# ODOMETRIE
# ============================================================

x = 0.0
y = 0.0
theta = 0.0

x_list = []
y_list = []

plot_update_counter = 0

last = time.time()

c_list = []


# ============================================================
# POINTS POUR LE DEFI 5
# ============================================================
#
# Les points seront conserves EXACTEMENT dans l'ordre
# dans lequel ils sont mesures.
#
# Format :
# (x, y, theta, couleur)
#
# Les coordonnees sont en metres et adaptees au repere
# que l'on reutilisera ensuite avec go_to_xya().
# ============================================================

points_parcours = []


# ============================================================
# VARIABLES POUR LE RECALAGE DES REPÈRES
# ============================================================

couleur_repere_actuelle = None

# Origine du troncon dans la carte BRUTE
origine_brute_x = 0.0
origine_brute_y = 0.0

# Meme origine dans la carte CORRIGEE
origine_corrigee_x = 0.0
origine_corrigee_y = 0.0

# Dernier point avant le changement de couleur
dernier_brut_x = 0.0
dernier_brut_y = 0.0

dernier_corrige_x = 0.0
dernier_corrige_y = 0.0

premiere_mesure = True


# ============================================================
# ROTATION D'UN VECTEUR 2D
# ============================================================

def rotation_point(dx, dy, angle_deg):

    angle = math.radians(angle_deg)

    dx_rotation = (
        dx * math.cos(angle)
        - dy * math.sin(angle)
    )

    dy_rotation = (
        dx * math.sin(angle)
        + dy * math.cos(angle)
    )

    return dx_rotation, dy_rotation


# ============================================================
# PASSAGE DES DIFFERENTS CIRCUITS DANS UN REPERE COMMUN
# ============================================================

def corriger_repere(x_brut, y_brut, couleur):

    global couleur_repere_actuelle

    global origine_brute_x
    global origine_brute_y

    global origine_corrigee_x
    global origine_corrigee_y

    global dernier_brut_x
    global dernier_brut_y

    global dernier_corrige_x
    global dernier_corrige_y

    global premiere_mesure


    angle_deg = rotation_repere_deg.get(couleur, 0.0)


    # ========================================================
    # PREMIERE MESURE DU PARCOURS
    # ========================================================

    if premiere_mesure:

        couleur_repere_actuelle = couleur

        # Le robot part de l'origine du repere monde
        origine_brute_x = 0.0
        origine_brute_y = 0.0

        origine_corrigee_x = 0.0
        origine_corrigee_y = 0.0

        premiere_mesure = False


    # ========================================================
    # CHANGEMENT DE COULEUR
    # ========================================================

    elif couleur != couleur_repere_actuelle:

        print()
        print(
            f"--- Changement de circuit : "
            f"{couleur_repere_actuelle} -> {couleur} ---"
        )

        print(
            f"Correction du nord : "
            f"{angle_deg:.1f} deg"
        )


        # ----------------------------------------------------
        # Il y a un petit déplacement entre :
        #
        # - la derniere mesure de l'ancien circuit
        # - la premiere mesure du nouveau
        #
        # On ne veut surtout pas le perdre.
        # ----------------------------------------------------

        dx_transition = x_brut - dernier_brut_x
        dy_transition = y_brut - dernier_brut_y


        dx_transition_corrige, dy_transition_corrige = rotation_point(
            dx_transition,
            dy_transition,
            angle_deg
        )


        # Le premier point du nouveau circuit continue
        # exactement depuis le dernier point de l'ancien.
        origine_corrigee_x = (
            dernier_corrige_x
            + dx_transition_corrige
        )

        origine_corrigee_y = (
            dernier_corrige_y
            + dy_transition_corrige
        )


        # Le point actuel devient l'origine LOCALE
        # du nouveau circuit.
        origine_brute_x = x_brut
        origine_brute_y = y_brut


        couleur_repere_actuelle = couleur


    # ========================================================
    # POSITION DU POINT PAR RAPPORT AU DEBUT DU TRONCON
    # ========================================================

    dx = x_brut - origine_brute_x
    dy = y_brut - origine_brute_y


    # ========================================================
    # ROTATION DU TRONCON
    # ========================================================

    dx_corrige, dy_corrige = rotation_point(
        dx,
        dy,
        angle_deg
    )


    # ========================================================
    # RETOUR DANS LE REPERE GLOBAL
    # ========================================================

    x_corrige = (
        origine_corrigee_x
        + dx_corrige
    )

    y_corrige = (
        origine_corrigee_y
        + dy_corrige
    )


    # ========================================================
    # MEMORISATION POUR LA PROCHAINE MESURE
    # ========================================================

    dernier_brut_x = x_brut
    dernier_brut_y = y_brut

    dernier_corrige_x = x_corrige
    dernier_corrige_y = y_corrige


    return x_corrige, y_corrige


# ============================================================
# ODOMETRIE
# ============================================================

def odometry(period=0.02):

    global x
    global y
    global theta

    global x_list
    global y_list

    global plot_update_counter
    global last

    global numeroCouleur
    global c_list

    global points_parcours


    # ========================================================
    # DT REEL MESURE
    # ========================================================

    now = time.time()

    dt = now - last

    last = now


    # ========================================================
    # LECTURE DES VITESSES DES ROUES
    # ========================================================

    speeds = dxl_io.get_present_speed([1, 2])

    v_left = speeds[0]
    v_right = -speeds[1]


    # ========================================================
    # CINEMATIQUE DIRECTE
    # ========================================================

    x_dot, theta_dot = direct_kinematics(
        v_left,
        v_right
    )


    # ========================================================
    # ODOMETRIE
    #
    # IMPORTANT :
    # je garde EXACTEMENT votre ordre actuel :
    #
    # y, x, theta = tick_odom(y, x, ...)
    #
    # Je ne modifie pas ce qui fonctionne deja.
    # ========================================================

    y, x, theta = tick_odom(
        y,
        x,
        theta,
        x_dot,
        theta_dot,
        dt
    )


    # ========================================================
    # COORDONNEES DE VOTRE CARTE ACTUELLE
    #
    # Avant vous faisiez :
    #
    # x_cm = -x * 100
    # y_cm = y * 100
    #
    # On garde EXACTEMENT ce repere.
    # On travaille simplement en metres avant affichage.
    # ========================================================

    x_brut = -x
    y_brut = y


    # ========================================================
    # RECALAGE DU "NORD" DU TRONCON
    # ========================================================

    x_corrige, y_corrige = corriger_repere(
        x_brut,
        y_brut,
        numeroCouleur
    )


    # ========================================================
    # THETA CORRIGE
    # ========================================================

    angle_correction = math.radians(
        rotation_repere_deg.get(
            numeroCouleur,
            0.0
        )
    )


    theta_corrige = theta + angle_correction


    # Normalisation dans [-pi, pi]
    theta_corrige = math.atan2(
        math.sin(theta_corrige),
        math.cos(theta_corrige)
    )


    # ========================================================
    # POINTS POUR LE DEFI 5
    #
    # La carte affiche :
    #
    # Xcarte = -Ygoto
    # Ycarte =  Xgoto
    #
    # Donc pour retrouver le repere de go_to_xya :
    #
    # Xgoto = Ycarte
    # Ygoto = -Xcarte
    # ========================================================

    x_goto = y_corrige
    y_goto = -x_corrige


    points_parcours.append(
        (
            x_goto,
            y_goto,
            theta_corrige,
            numeroCouleur
        )
    )


    # ========================================================
    # CONVERSION EN CM POUR LE GRAPHIQUE
    # ========================================================

    x_cm = x_corrige * 100
    y_cm = y_corrige * 100


    x_list.append(x_cm)
    y_list.append(y_cm)


    # ========================================================
    # COULEUR DU POINT
    # ========================================================

    map_colors = {
        0: 'yellow',
        1: 'blue',
        2: 'red'
    }


    c_list.append(
        map_colors.get(
            numeroCouleur,
            'black'
        )
    )


    # ========================================================
    # AFFICHAGE TERMINAL
    # ========================================================

    print(
        f"\r"
        f"X: {x_cm:7.1f} cm | "
        f"Y: {y_cm:7.1f} cm | "
        f"Cap: {math.degrees(theta_corrige):7.1f}°",
        end=""
    )


    # ========================================================
    # MISE A JOUR DU GRAPHIQUE
    # ========================================================

    traj.set_offsets(
        np.c_[x_list, y_list]
    )

    traj.set_facecolors(c_list)
    traj.set_edgecolors(c_list)

    robot.set_data(
        x_list[-1:],
        y_list[-1:]
    )

    ax.update_datalim(
        np.c_[x_list, y_list]
    )

    ax.autoscale_view()

    plt.pause(0.001)


    return (
        x,
        y,
        theta,
        x_list,
        y_list,
        plot_update_counter,
        last
    )


# ============================================================
# DECLARATION HSV DES COULEURS
# ============================================================

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


numeroCouleur = 0

# Delai avant detection du vert
attenteVert = 90

# Nombre de frames consecutives avec du vert
compteurVert = 0

# Premiere couleur : jaune
lower = lower_yellow
upper = upper_yellow


# ============================================================
# PREDICTION
# ============================================================

def prediction(img):

    global numeroCouleur
    global lower
    global upper
    global attenteVert
    global compteurVert


    hsv_img = cv2.cvtColor(
        img,
        cv2.COLOR_BGR2HSV
    )


    attenteVert -= 1


    # ========================================================
    # DETECTION DU VERT
    # ========================================================

    if attenteVert <= 0:

        mask = cv2.inRange(
            hsv_img,
            lower_green,
            upper_green
        )


        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        if contours:

            largestContour = max(
                contours,
                key=cv2.contourArea
            )


            moment = cv2.moments(
                largestContour
            )


            if moment["m00"] > 800:

                print(compteurVert)

                compteurVert += 1


                if compteurVert >= 2:

                    attenteVert = 180

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

                            # rouge gere plus bas
                            print("rouge")


                        case 3:
                            odometry()
                            rotate_and_move(0, 1)

                            time.sleep(0.25)

                            stop()

                            exit()


                    print(lower)


            else:

                compteurVert = 0


        else:

            compteurVert = 0


    # ========================================================
    # MASQUE DE LA COULEUR ACTUELLE
    # ========================================================

    if numeroCouleur == 2:

        mask1 = cv2.inRange(
            hsv_img,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv_img,
            lower_red2,
            upper_red2
        )

        mask = cv2.bitwise_or(
            mask1,
            mask2
        )


    else:

        mask = cv2.inRange(
            hsv_img,
            lower,
            upper
        )


    # ========================================================
    # CONTOURS
    # ========================================================

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )


    if contours:

        largestContour = max(
            contours,
            key=cv2.contourArea
        )


        moment = cv2.moments(
            largestContour
        )


        if moment["m00"] > 300:

            cx = int(
                moment["m10"]
                / moment["m00"]
            )


            centreImage = (
                img.shape[1] / 2
            )


            erreur = (
                cx
                - centreImage
            )


            direction = int(
                (erreur / centreImage)
                * 100
            )


            direction = max(
                -100,
                min(
                    100,
                    direction
                )
            )


            return direction


    # Si aucune ligne n'est trouvee :
    # on va tout droit.
    return 0


# ============================================================
# SAUVEGARDE DES POINTS POUR LE DEFI 5
# ============================================================

def sauvegarder_points():

    donnees = []


    for numero, point in enumerate(
        points_parcours
    ):

        px, py, ptheta, couleur = point


        donnees.append(
            {
                "numero": numero,
                "x": px,
                "y": py,
                "theta": ptheta,
                "couleur": couleur
            }
        )


    with open(
        "points_parcours.json",
        "w"
    ) as fichier:

        json.dump(
            donnees,
            fichier,
            indent=4
        )


    print(
        f"\n{len(points_parcours)} points sauvegardes "
        f"dans points_parcours.json"
    )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    frameCounter = 0


    # ========================================================
    # CAMERA
    # ========================================================

    cap = cv2.VideoCapture(0)


    if not cap.isOpened():

        print(
            "Error: Could not open webcam."
        )

        exit()


    # ========================================================
    # GRAPHIQUE
    # ========================================================

    fig, ax = plt.subplots(
        figsize=(8, 8)
    )


    traj = ax.scatter(
        [],
        [],
        label="Trajectoire"
    )


    robot, = ax.plot(
        x_list[-1:],
        y_list[-1:],
        'ro',
        label="Robot"
    )


    try:

        # ====================================================
        # BOUCLE CAMERA
        # ====================================================

        while True:

            ret, frame = cap.read()


            if not ret:

                print(
                    "Error: Can't receive frame."
                )

                break


            # Resolution plus legere
            frame = cv2.resize(
                frame,
                (360, 240)
            )


            # On garde le bas de l'image
            frame = frame[100:360, :]


            # =================================================
            # PREDICTION
            # =================================================

            direction = prediction(
                frame
            )


            # =================================================
            # MOTEURS
            #
            # IMPORTANT :
            #
            # C'EST EXACTEMENT TA LIGNE D'ORIGINE.
            #
            # On reste TOUJOURS avec numeroCouleur = 2
            # pour la vitesse moteur.
            #
            # Je ne change PAS la vitesse du robot.
            # =================================================

            rotate_and_move(
                direction,
                2
            )


            # =================================================
            # ODOMETRIE TOUTES LES 20 FRAMES
            #
            # EXACTEMENT COMME TON CODE.
            # =================================================

            frameCounter += 1


            if frameCounter >= 20:

                odometry()

                frameCounter = 0


            # =================================================
            # Q POUR QUITTER
            # =================================================

            if (
                cv2.waitKey(1)
                & 0xFF
                == ord('q')
            ):

                plt.show()

                stop()

                break


    finally:

        stop()

        cap.release()

        cv2.destroyAllWindows()


        # ====================================================
        # GRAPHIQUE FINAL
        # ====================================================

        ax.set_title(
            "Cartographie par Odométrie Temps Réel"
        )

        ax.set_xlabel(
            "X (cm)"
        )

        ax.set_ylabel(
            "Y (cm)"
        )

        ax.grid(True)

        ax.legend()

        ax.axis('equal')


        plt.savefig(
            "trajectoire_parcours.png"
        )


        print(
            "\nGraphique sauvegarde sous "
            "'trajectoire_parcours.png'."
        )


        # ====================================================
        # POINTS POUR LE DEFI 5
        # ====================================================

        sauvegarder_points()
