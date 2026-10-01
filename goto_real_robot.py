import math
import time
import pypot.dynamixel


# ---------- Paramètres du robot ----------

R = 0.025
L_COMMAND = 0.146   # entraxe géométrique
L_ODOM = 0.153      # entraxe calibré pour l'odométrie

PERIOD = 0.02

POSITION_TOL = 0.005
ANGLE_TOL = math.radians(0.7)
HEADING_TOL = math.radians(1.5)

MAX_LINEAR = 0.12
MAX_ANGULAR = 0.75

ACCEL = 12.0        # rad/s²
DECEL = 28.0
MAX_MOTOR = math.radians(750)


# ---------- Connexion Dynamixel ----------

ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise SystemExit("Aucun port Dynamixel détecté")

dxl = pypot.dynamixel.DxlIO(ports[0])
dxl.set_wheel_mode([1, 2])


# ---------- Pose estimée ----------

x = 0.0
y = 0.0
theta = 0.0

cmd_left = 0.0
cmd_right = 0.0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalise_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------- Cinématique ----------

def inverse_kinematics(v, w):
    """Vitesse du robot -> vitesses des roues."""
    left = (v - w * L_COMMAND / 2.0) / R
    right = (v + w * L_COMMAND / 2.0) / R
    return left, right


def direct_kinematics(left, right):
    """Vitesses des roues -> vitesse du robot."""
    v = R * (left + right) / 2.0
    w = R * (right - left) / L_ODOM
    return v, w


# ---------- Odométrie ----------

def update_odometry(dt):
    global x, y, theta

    speeds = dxl.get_present_speed([1, 2])

    # Les moteurs sont montés en miroir.
    left = math.radians(speeds[0])
    right = math.radians(-speeds[1])

    v, w = direct_kinematics(left, right)
    dtheta = w * dt

    if abs(w) > 1e-6:
        radius = v / w
        dx_local = radius * math.sin(dtheta)
        dy_local = radius * (1 - math.cos(dtheta))
    else:
        dx_local = v * dt
        dy_local = 0.0

    x += dx_local * math.cos(theta) - dy_local * math.sin(theta)
    y += dx_local * math.sin(theta) + dy_local * math.cos(theta)
    theta = normalise_angle(theta + dtheta)


# ---------- Commande des moteurs ----------

def ramp(current, target, dt):
    """Limite les changements brusques de vitesse."""
    rate = DECEL if abs(target) < abs(current) or current * target < 0 else ACCEL
    step = rate * dt
    return clamp(target, current - step, current + step)


def set_robot_speed(v, w, dt):
    global cmd_left, cmd_right

    left_target, right_target = inverse_kinematics(v, w)

    cmd_left = ramp(cmd_left, left_target, dt)
    cmd_right = ramp(cmd_right, right_target, dt)

    cmd_left = clamp(cmd_left, -MAX_MOTOR, MAX_MOTOR)
    cmd_right = clamp(cmd_right, -MAX_MOTOR, MAX_MOTOR)

    dxl.set_moving_speed({
        1: math.degrees(cmd_left),
        2: -math.degrees(cmd_right)
    })


def stop_robot():
    """Freine progressivement tout en continuant l'odométrie."""
    global cmd_left, cmd_right

    last = time.monotonic()

    while abs(cmd_left) > 1e-3 or abs(cmd_right) > 1e-3:
        start = time.monotonic()
        dt = max(start - last, 1e-4)
        last = start

        update_odometry(dt)
        set_robot_speed(0.0, 0.0, dt)

        time.sleep(max(0.0, PERIOD - (time.monotonic() - start)))

    dxl.set_moving_speed({1: 0, 2: 0})


# ---------- Aller à une position ----------

def go_to_xya(target_x, target_y, target_theta):
    """
    1. S'aligner vers le point.
    2. Avancer vers le point.
    3. Prendre l'orientation finale.
    """

    phase = "align"
    last = time.monotonic()

    while True:
        start = time.monotonic()
        dt = max(start - last, 1e-4)
        last = start

        update_odometry(dt)

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        # Position atteinte : orientation finale
        if distance <= POSITION_TOL:
            if phase != "final":
                stop_robot()
                phase = "final"
                last = time.monotonic()
                continue

            error = normalise_angle(target_theta - theta)

            if abs(error) <= ANGLE_TOL:
                stop_robot()
                return

            w = clamp(1.6 * error, -MAX_ANGULAR, MAX_ANGULAR)
            set_robot_speed(0.0, w, dt)

        else:
            target_heading = math.atan2(dy, dx)
            error = normalise_angle(target_heading - theta)

            # Phase 1 : s'aligner avant d'avancer
            if phase == "align":
                if abs(error) <= ANGLE_TOL:
                    stop_robot()
                    phase = "drive"
                    last = time.monotonic()
                    continue

                w = clamp(2.0 * error, -MAX_ANGULAR, MAX_ANGULAR)
                set_robot_speed(0.0, w, dt)

            # Phase 2 : avancer presque droit
            else:
                if abs(error) > HEADING_TOL:
                    stop_robot()
                    phase = "align"
                    last = time.monotonic()
                    continue

                if distance > 0.08:
                    v = 0.12
                elif distance > 0.025:
                    v = 0.075
                else:
                    v = 0.04

                w = clamp(1.2 * error, -0.20, 0.20)
                set_robot_speed(v, w, dt)

        time.sleep(max(0.0, PERIOD - (time.monotonic() - start)))


# ---------- Test ----------

if __name__ == "__main__":
    try:
        dxl.set_moving_speed({1: 0, 2: 0})
        time.sleep(1)

        # Triangle rectangle
        go_to_xya(0.50, 0.00, math.radians(0))
        go_to_xya(0.50, 0.00, math.radians(180))
        go_to_xya(0.00, 0.00, math.radians(0))

    finally:
        dxl.set_moving_speed({1: 0, 2: 0})
