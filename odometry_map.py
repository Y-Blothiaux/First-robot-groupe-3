import math
import time
from goto import dxl_io, direct_kinematics, tick_odom
import matplotlib.pyplot as plt

def odometry(period=0.02):
    fig, ax = plt.subplots(figsize=(8, 8))

    x_list = []
    y_list = []

    line, = ax.plot(x_list, y_list)
    robot, = ax.plot(x_list[-1:], y_list[-1:])

    ax.set_title("Cartographie par Odométrie Temps Réel")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.grid(True)
    ax.legend()
    ax.axis('equal') # Échelle 1:1 pour ne pas déformer la trajectoire

    # Roues libres, on peut pousser le robot à la main
    dxl_io.disable_torque([1, 2])
    x = 0.0
    y = 0.0
    theta = 0.0
    last = time.time()

    plot_update_counter = 0 # Pour mettre à jour le plot de manière synchrone avec la boucle (toutes les 5 itérations)

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
            y, x, theta = tick_odom(y, x, theta, x_dot, theta_dot, dt)

            x_cm = -x * 100
            y_cm = y * 100

            x_list.append(x_cm)
            y_list.append(y_cm)

            print(f"\rX: {x_cm:7.1f} cm | Y: {y_cm:7.1f} cm | Cap: {math.degrees(theta):7.1f}°", end="")

            plot_update_counter += 1
            if plot_update_counter % 5:
                line.set_data(x_list, y_list)
                robot.set_data(x_list[-1:], y_list[-1:])
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.001) # Pause nécessaire à la MAJ

            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\nPosition finale -> X: {x_cm:.1f} cm | Y: {y_cm:.1f} cm | Cap: {math.degrees(theta):.1f}°")
        print("\nArrêt du suivi odométrique.")

        plt.show()

if __name__ == "__main__":
    odometry()









