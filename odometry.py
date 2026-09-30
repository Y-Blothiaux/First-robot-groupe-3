import math
import pypot.dynamixel
import time
from goto import direct_kinematics

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])


def odometry(dt):
    global x_inUse, y_inUse, theta_inUse

    dxl_io.disable_torque([1, 2])
    x_inUse = 0.0
    y_inUse = 0.0
    theta_inUse = 0.0

    try:
        while True:
            # On lit la vitesse des roues poussées
            speeds = dxl_io.get_present_speed([1, 2])
            v_left_inUse = speeds[0]
            v_right_inUse = -speeds[1]
            x_dot, theta_dot = direct_kinematics(v_left_inUse, v_right_inUse)
            x_inUse, y_inUse, theta_inUse = tick_odom(x_inUse, y_inUse, theta_inUse, x_dot, theta_dot, dt)
            # On affiche la position           
            angle_deg = math.degrees(theta_inUse)
            print(f"Position -> X: {x_inUse*100:.1f} cm | Y: {y_inUse*100:.1f} cm | Cap: {angle_deg:.1f}°")
            time.sleep(dt)
    except KeyboardInterrupt:
        # Permet de sortir de la boucle infinie en appuyant sur Ctrl+C dans le terminal
        print("\nArrêt du suivi odométrique.")













