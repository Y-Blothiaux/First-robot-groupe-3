import math
import pypot.dynamixel
import time


R = 0.025  # rayon de la roue = 2.5 cm
L = 0.146  # distance entre les roues = 14.6 cm


# Variables de position du robot dans le monde
x_inUse = 0.0
y_inUse = 0.0
theta_inUse = 0.0

x_dot_inUse = 0.0
theta_dot_inUse = 0.0

# Période CIBLE de la boucle.
# IMPORTANT : ce n'est plus utilisé comme dt "supposé" de l'odométrie.
dt_inUse = 0.02


# Initialisation
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


def inverse_kinematics(x_dot, theta_dot):
    v_left_rad = (x_dot - (theta_dot * L / 2.0)) / R
    v_right_rad = (x_dot + (theta_dot * L / 2.0)) / R

    # Les Dynamixel sont commandés ici en degrés/seconde.
    v_left_deg = math.degrees(v_left_rad)
    v_right_deg = math.degrees(v_right_rad)

    return v_left_deg, v_right_deg


def direct_kinematics(v_left, v_right):
    # get_present_speed() renvoie ici des degrés/seconde.
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
    i, j, k = odom2D(x_dot, theta_dot, dt)

    new_x = x + (i * math.cos(theta) - j * math.sin(theta))
    new_y = y + (i * math.sin(theta) + j * math.cos(theta))
    new_theta = normalise_angle(theta + k)

    return new_x, new_y, new_theta


def normalise_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def go_to_xya(target_x, target_y, target_theta, period=0.02):
    """
    Version corrigée en restant volontairement proche du goto original.

    Corrections principales :
      1. dt réellement mesuré, comme dans odometry.py.
      2. tolérance de position réduite de 3 cm à 1 cm.
      3. suppression de la tolérance de cap de 0.25 rad près de la cible.
      4. vitesse minimale réduite près de la cible.
      5. period est une période de boucle visée, pas un dt supposé.
    """

    global x_inUse, y_inUse, theta_inUse

    # FIX 1 : 3 cm autorisait explicitement le robot à ne pas atteindre le sommet.
    target_diff_dist_max = 0.01       # 1 cm

    # Environ 1.15 degré.
    target_diff_angle_max = 0.02

    # Environ 1.7 degré.
    tolerated_cap_steps = 0.03

    vitesse_marche = 0.15
    speed_rotation = 3.0

    # FIX 2 : on mesure le vrai temps écoulé entre deux mises à jour.
    last_time = time.monotonic()

    while True:
        loop_start = time.monotonic()

        now = loop_start
        dt_reel = now - last_time
        last_time = now

        # Evite un dt quasi nul au tout premier passage.
        if dt_reel <= 0.0:
            dt_reel = period

        # Lecture des vitesses réelles des moteurs.
        speeds = dxl_io.get_present_speed([1, 2])
        v_left_inUse = speeds[0]

        # Moteur droit monté en miroir sur le robot physique.
        v_right_inUse = -speeds[1]

        x_dot_inUse, theta_dot_inUse = direct_kinematics(
            v_left_inUse,
            v_right_inUse
        )

        # FIX 3 : utilisation du dt REEL, pas de 0.02 supposé.
        x_inUse, y_inUse, theta_inUse = tick_odom(
            x_inUse,
            y_inUse,
            theta_inUse,
            x_dot_inUse,
            theta_dot_inUse,
            dt_reel
        )

        dx = target_x - x_inUse
        dy = target_y - y_inUse
        distance_togo = math.sqrt(dx**2 + dy**2)

        if distance_togo > target_diff_dist_max:
            angle_to_point = math.atan2(dy, dx)
            error_cap = normalise_angle(angle_to_point - theta_inUse)

            # FIX 4 :
            # On NE PASSE PLUS à 0.25 rad (~14.3°) quand on est proche.
            # On garde une tolérance faible jusqu'au point cible.
            tolerance_actuelle = tolerated_cap_steps

            # FIX 5 :
            # ralentissement progressif et minimum plus faible (2 cm/s).
            vitesse_avance = max(
                0.02,
                min(vitesse_marche, 1.5 * distance_togo)
            )

            if abs(error_cap) > tolerance_actuelle:
                # On s'aligne d'abord.
                x_dot_target = 0.0
                theta_dot_target = max(
                    -speed_rotation,
                    min(speed_rotation, 2.0 * error_cap)
                )
            else:
                # Puis on avance.
                x_dot_target = vitesse_avance
                theta_dot_target = 0.0

        else:
            # Position atteinte : orientation finale.
            error_angle_final = normalise_angle(
                target_theta - theta_inUse
            )

            if abs(error_angle_final) > target_diff_angle_max:
                x_dot_target = 0.0
                theta_dot_target = max(
                    -speed_rotation,
                    min(speed_rotation, 2.0 * error_angle_final)
                )
            else:
                dxl_io.set_moving_speed({1: 0, 2: 0})

                print(
                    f"Cible atteinte -> "
                    f"X={x_inUse:.3f} m | "
                    f"Y={y_inUse:.3f} m | "
                    f"Cap={math.degrees(theta_inUse):.1f}°"
                )
                break

        v_left_deg, v_right_deg = inverse_kinematics(
            x_dot_target,
            theta_dot_target
        )

        dxl_io.set_moving_speed({
            1: v_left_deg,
            2: -v_right_deg
        })

        # period=20 ms est la période VISÉE.
        # Contrairement à time.sleep(0.02) systématique, on retire le temps
        # déjà consommé par les communications/calculs.
        elapsed = time.monotonic() - loop_start
        sleep_time = period - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    # S'assurer que le robot est à l'arrêt avant de commencer.
    dxl_io.set_moving_speed({1: 0, 2: 0})
    time.sleep(1)

    # (0, 0) -> (0.5, 0)
    go_to_xya(0.5, 0.0, 0.0, dt_inUse)

    time.sleep(2)

    # (0.5, 0) -> (0.5, 0.5)
    go_to_xya(0.5, 0.5, math.radians(90), dt_inUse)

    time.sleep(2)

    # retour vers l'origine
    go_to_xya(0.0, 0.0, math.radians(-135), dt_inUse)

    # Arrêt de sécurité.
    dxl_io.set_moving_speed({1: 0, 2: 0})
