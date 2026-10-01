import math
import time
import pypot.dynamixel


# Géométrie / calibration
R = 0.025
L_COMMAND = 0.146   # entraxe géométrique
L_ODOM = 0.153      # entraxe effectif calibré pour l'odométrie

PERIOD = 0.02

# Réglages de déplacement
POSITION_TOL = 0.005
ANGLE_TOL = math.radians(0.7)
DRIVE_HEADING_TOL = math.radians(1.5)

MAX_LINEAR_SPEED = 0.12
MAX_ANGULAR_SPEED = 0.75

MAX_WHEEL_ACCEL = 12.0   # rad/s²
MAX_WHEEL_DECEL = 28.0   # rad/s²
MAX_MOTOR_SPEED = math.radians(750)  # limite de sécurité

# Pose estimée
x_inUse = 0.0
y_inUse = 0.0
theta_inUse = 0.0

# Commandes courantes des roues en rad/s
command_left = 0.0
command_right = 0.0


# Connexion Dynamixel
ports = pypot.dynamixel.get_available_ports()
if not ports:
    raise SystemExit("Aucun port Dynamixel détecté")

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


def normalise_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inverse_kinematics(x_dot, theta_dot):
    """Vitesse robot -> vitesses des roues en rad/s."""
    left = (x_dot - theta_dot * L_COMMAND / 2.0) / R
    right = (x_dot + theta_dot * L_COMMAND / 2.0) / R
    return left, right


def direct_kinematics(left, right):
    """Vitesses des roues en rad/s -> vitesse du robot."""
    x_dot = R * (left + right) / 2.0
    theta_dot = R * (right - left) / L_ODOM
    return x_dot, theta_dot


def tick_odom(x, y, theta, x_dot, theta_dot, dt):
    dtheta = theta_dot * dt

    if abs(theta_dot) > 1e-6:
        radius = x_dot / theta_dot
        dx_local = radius * math.sin(dtheta)
        dy_local = radius * (1.0 - math.cos(dtheta))
    else:
        dx_local = x_dot * dt
        dy_local = 0.0

    new_x = x + dx_local * math.cos(theta) - dy_local * math.sin(theta)
    new_y = y + dx_local * math.sin(theta) + dy_local * math.cos(theta)

    return new_x, new_y, normalise_angle(theta + dtheta)


def update_odometry(dt):
    """Met à jour la pose avec les vitesses réellement lues sur les moteurs."""
    global x_inUse, y_inUse, theta_inUse

    speeds = dxl_io.get_present_speed([1, 2])

    # Les deux moteurs sont montés en miroir.
    left = math.radians(speeds[0])
    right = math.radians(-speeds[1])

    x_dot, theta_dot = direct_kinematics(left, right)

    x_inUse, y_inUse, theta_inUse = tick_odom(
        x_inUse, y_inUse, theta_inUse,
        x_dot, theta_dot, dt
    )


def move_towards(current, target, dt):
    reducing = abs(target) < abs(current) or current * target < 0
    rate = MAX_WHEEL_DECEL if reducing else MAX_WHEEL_ACCEL
    step = rate * dt

    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def set_wheel_speeds(left_target, right_target, dt):
    """Applique une rampe pour éviter les changements trop brusques."""
    global command_left, command_right

    command_left = move_towards(command_left, left_target, dt)
    command_right = move_towards(command_right, right_target, dt)

    command_left = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, command_left))
    command_right = max(-MAX_MOTOR_SPEED, min(MAX_MOTOR_SPEED, command_right))

    dxl_io.set_moving_speed({
        1: math.degrees(command_left),
        2: -math.degrees(command_right)
    })


def set_robot_velocity(x_dot, theta_dot, dt):
    left, right = inverse_kinematics(x_dot, theta_dot)
    set_wheel_speeds(left, right, dt)


def brake():
    """Freine en continuant l'odométrie jusqu'à l'arrêt."""
    global command_left, command_right

    last = time.monotonic()

    while abs(command_left) > 1e-3 or abs(command_right) > 1e-3:
        start = time.monotonic()
        dt = start - last
        last = start

        update_odometry(dt)
        set_wheel_speeds(0.0, 0.0, dt)

        remaining = PERIOD - (time.monotonic() - start)
        if remaining > 0:
            time.sleep(remaining)

    dxl_io.set_moving_speed({1: 0, 2: 0})


def go_to_xya(target_x, target_y, target_theta):
    """Va au point (x, y), puis prend l'orientation finale demandée."""
    state = "ALIGN"
    last = time.monotonic()

    while True:
        start = time.monotonic()
        dt = start - last
        last = start

        if dt <= 0:
            dt = PERIOD

        update_odometry(dt)

        dx = target_x - x_inUse
        dy = target_y - y_inUse
        distance = math.hypot(dx, dy)

        # Position atteinte : orientation finale
        if distance <= POSITION_TOL:
            if state != "FINAL":
                brake()
                state = "FINAL"
                last = time.monotonic()

            error = normalise_angle(target_theta - theta_inUse)

            if abs(error) <= ANGLE_TOL:
                brake()
                return

            angular = max(
                -MAX_ANGULAR_SPEED,
                min(MAX_ANGULAR_SPEED, 1.6 * error)
            )
            set_robot_velocity(0.0, angular, dt)

        else:
            desired_heading = math.atan2(dy, dx)
            heading_error = normalise_angle(desired_heading - theta_inUse)

            # 1) On s'aligne précisément avant d'avancer
            if state == "ALIGN":
                if abs(heading_error) <= ANGLE_TOL:
                    brake()
                    state = "DRIVE"
                    last = time.monotonic()
                    continue

                angular = max(
                    -MAX_ANGULAR_SPEED,
                    min(MAX_ANGULAR_SPEED, 2.0 * heading_error)
                )
                set_robot_velocity(0.0, angular, dt)

            # 2) On avance presque droit ; si le cap dérive trop, on se réaligne
            else:
                if abs(heading_error) > DRIVE_HEADING_TOL:
                    brake()
                    state = "ALIGN"
                    last = time.monotonic()
                    continue

                if distance > 0.08:
                    linear = 0.12
                elif distance > 0.025:
                    linear = 0.075
                else:
                    linear = 0.04

                angular = max(-0.20, min(0.20, 1.2 * heading_error))
                set_robot_velocity(linear, angular, dt)

        remaining = PERIOD - (time.monotonic() - start)
        if remaining > 0:
            time.sleep(remaining)


if __name__ == "__main__":
    try:
        dxl_io.set_moving_speed({1: 0, 2: 0})
        time.sleep(1)

        # Exemple : triangle rectangle
        go_to_xya(0.50, 0.00, math.radians(0))
        go_to_xya(0.50, 0.50, math.radians(90))
        go_to_xya(0.00, 0.00, math.radians(-135))

    finally:
        dxl_io.set_moving_speed({1: 0, 2: 0})
