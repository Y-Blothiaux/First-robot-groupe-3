import math
import time
import pypot.dynamixel


# ---------- Mesures du robot ----------
R = 0.025           # rayon des roues (m)
L_COMMAND = 0.146   # entraxe utilisé pour commander les roues (m)
L_ODOM = 0.153      # entraxe calibré pour l'odométrie (m)

# ---------- Réglages ----------
PERIOD = 0.02                          # durée d'un cycle (s)
POSITION_TOL = 0.005                   # précision en position (m)
ANGLE_TOL = math.radians(0.7)          # précision en angle (rad)
DRIVE_HEADING_TOL = math.radians(1.5)  # écart de cap max pendant qu'on avance

MAX_LINEAR_SPEED = 0.12    # m/s
MAX_ANGULAR_SPEED = 0.75   # rad/s
ACCEL_LIN = 0.4            # m/s²   (rampe de vitesse linéaire)
ACCEL_ANG = 5.0            # rad/s² (rampe de vitesse de rotation)

# ---------- État du robot ----------
x_inUse = 0.0
y_inUse = 0.0
theta_inUse = 0.0

v_now = 0.0   # vitesse linéaire actuellement commandée
w_now = 0.0   # vitesse de rotation actuellement commandée
last_time = time.monotonic()

# ---------- Connexion aux moteurs ----------
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise SystemExit("Aucun port Dynamixel détecté")

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


# ---------- Outils ----------
def normalise_angle(angle):
    """Ramène un angle dans [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    """Limite value entre -limit et +limit."""
    return max(-limit, min(limit, value))


def move_towards(current, target, max_step):
    """Rapproche current de target, de max_step au maximum."""
    if target > current + max_step:
        return current + max_step
    if target < current - max_step:
        return current - max_step
    return target


# ---------- Cinématique ----------
def inverse_kinematics(x_dot, theta_dot):
    """Vitesse du robot -> vitesses des roues (rad/s)."""
    left = (x_dot - theta_dot * L_COMMAND / 2.0) / R
    right = (x_dot + theta_dot * L_COMMAND / 2.0) / R
    return left, right


def direct_kinematics(left, right):
    """Vitesses des roues (rad/s) -> vitesse du robot."""
    x_dot = R * (left + right) / 2.0
    theta_dot = R * (right - left) / L_ODOM
    return x_dot, theta_dot


# ---------- Odométrie ----------
def tick_odom(x, y, theta, x_dot, theta_dot, dt):
    """Calcule la nouvelle position à partir des vitesses."""
    dtheta = theta_dot * dt
    # On avance dans la direction moyenne prise pendant ce petit pas de temps
    middle_angle = theta + dtheta / 2.0
    new_x = x + x_dot * dt * math.cos(middle_angle)
    new_y = y + x_dot * dt * math.sin(middle_angle)
    return new_x, new_y, normalise_angle(theta + dtheta)


def update_odometry(dt):
    """Lit les moteurs et met à jour la position."""
    global x_inUse, y_inUse, theta_inUse

    speeds = dxl_io.get_present_speed([1, 2])   # en degrés/s
    left = math.radians(speeds[0])
    right = math.radians(-speeds[1])            # moteur droit monté en miroir

    x_dot, theta_dot = direct_kinematics(left, right)
    x_inUse, y_inUse, theta_inUse = tick_odom(
        x_inUse, y_inUse, theta_inUse, x_dot, theta_dot, dt
    )


# ---------- Un cycle de contrôle ----------
def step(v_target, w_target):
    """Un cycle : mesure, rampe de vitesse, commande des roues, attente."""
    global v_now, w_now, last_time

    start = time.monotonic()
    dt = start - last_time
    last_time = start
    if dt <= 0 or dt > 0.1:
        dt = PERIOD

    # 1) Où suis-je ?
    update_odometry(dt)

    # 2) On change la vitesse progressivement (pas de coup sec)
    v_now = move_towards(v_now, v_target, ACCEL_LIN * dt)
    w_now = move_towards(w_now, w_target, ACCEL_ANG * dt)

    # 3) On envoie la commande aux moteurs
    left, right = inverse_kinematics(v_now, w_now)
    dxl_io.set_moving_speed({1: math.degrees(left), 2: -math.degrees(right)})

    # 4) On attend la fin du cycle
    remaining = PERIOD - (time.monotonic() - start)
    if remaining > 0:
        time.sleep(remaining)


def stop():
    """Freine doucement jusqu'à l'arrêt complet."""
    while abs(v_now) > 0.001 or abs(w_now) > 0.001:
        step(0.0, 0.0)
    dxl_io.set_moving_speed({1: 0, 2: 0})


# ---------- Les 3 mouvements ----------
def turn_to(target_angle):
    """Tourne sur place jusqu'à l'angle voulu."""
    while True:
        error = normalise_angle(target_angle - theta_inUse)
        if abs(error) <= ANGLE_TOL:
            break
        step(0.0, clamp(2.0 * error, MAX_ANGULAR_SPEED))
    stop()


def drive_to(target_x, target_y):
    """Avance en ligne droite vers le point voulu."""
    while True:
        dx = target_x - x_inUse
        dy = target_y - y_inUse
        distance = math.hypot(dx, dy)
        if distance <= POSITION_TOL:
            break

        direction = math.atan2(dy, dx)
        heading_error = normalise_angle(direction - theta_inUse)

        # Si le robot s'est trop décalé, on s'arrête et on se réaligne
        if abs(heading_error) > DRIVE_HEADING_TOL:
            stop()
            turn_to(direction)
            continue

        # On ralentit en approchant de la cible
        if distance > 0.08:
            speed = MAX_LINEAR_SPEED
        elif distance > 0.025:
            speed = 0.075
        else:
            speed = 0.04

        # Petite correction de trajectoire pendant qu'on avance
        step(speed, clamp(1.2 * heading_error, 0.2))
    stop()


def go_to_xya(target_x, target_y, target_theta):
    """Va au point (x, y), puis prend l'orientation finale."""
    distance = math.hypot(target_x - x_inUse, target_y - y_inUse)
    if distance > POSITION_TOL:
        turn_to(math.atan2(target_y - y_inUse, target_x - x_inUse))
        drive_to(target_x, target_y)
    turn_to(target_theta)


if __name__ == "__main__":
    try:
        dxl_io.set_moving_speed({1: 0, 2: 0})
        time.sleep(1)

        # Exemple : triangle rectangle
        go_to_xya(0.50, 0.00, math.radians(0))
        go_to_xya(0.50, 0.00, math.radians(180))
        go_to_xya(0.00, 0.00, math.radians(0))

    finally:
        dxl_io.set_moving_speed({1: 0, 2: 0})
