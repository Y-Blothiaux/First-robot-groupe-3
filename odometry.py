import math
import time
from goto import dxl_io, direct_kinematics, tick_odom

def odometry(period=0.02):
    # Roues libres, on peut pousser le robot à la main
    dxl_io.disable_torque([1, 2])
    x = 0.0
    y = 0.0
    theta = 0.0
    last = time.time()

    try:
        while True:
            # dt réel mesuré
            now = time.time()
            dt = now - last
            last = now

            # On lit la vitesse des roues poussées
            speeds = dxl_io.get_present_speed([1, 2]) # en deg/s
            v_left = speeds[0]
            v_right = -speeds[1]
            x_dot, theta_dot = direct_kinematics(v_left, v_right)
            x, y, theta = tick_odom(x, y, theta, x_dot, theta_dot, dt)
            print(f"\rX: {y*100:7.1f} cm | Y: {x*100:7.1f} cm | Cap: {math.degrees(theta):7.1f}°", end="")
            time.sleep(period)
    except KeyboardInterrupt:
            print(f"\nPosition finale -> X: {y*100:.1f} cm | Y: {x*100:.1f} cm | Cap: {math.degrees(theta):.1f}°")
            print("\nArrêt du suivi odométrique.")

if __name__ == "__main__":
    odometry()











