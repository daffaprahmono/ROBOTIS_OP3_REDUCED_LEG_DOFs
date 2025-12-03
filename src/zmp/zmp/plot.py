import matplotlib.pyplot as plt
import matplotlib.patches as patches
from zmp import DynamicComCalculator
import rclpy
import time

def plot_dynamic_zmp_area():
    rclpy.init()
    node = DynamicComCalculator()

    # Tunggu data sensor tersedia
    while (node.accel is None or node.joint_positions is None or node.gyro_raw is None):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Hitung CoM dan ZMP awal
    com_data = node.calculate_dynamic_com()
    if not com_data:
        print("Gagal menghitung CoM awal.")
        return
    CoM_x, CoM_y, CoM_z = com_data

    zmp_data = node.calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)
    if not zmp_data:
        print("Gagal menghitung ZMP awal.")
        return
    ZMP_x, ZMP_y = zmp_data

    # Hitung Area Support Polygon sekali saja
    joint_states = [node.joint_positions]
    imu_accel_z = node.accel[1]
    polygon_data = node.calculate_support_polygon(joint_states, imu_accel_z)
    luas = polygon_data["area"]

    # Hitung dimensi segiempat berdasarkan area (dengan rasio 1.5:1)
    r = 1.5
    lebar = (luas / r) ** 0.5
    panjang = r * lebar
    bottom_left_x = ZMP_x - panjang / 2
    bottom_left_y = ZMP_y - lebar / 2

    # Inisialisasi plot (sekali saja)
    plt.ion()
    fig, ax = plt.subplots()
    rect = patches.Rectangle((bottom_left_x, bottom_left_y), panjang, lebar,
                             linewidth=2, edgecolor='blue', facecolor='lightblue')
    ax.add_patch(rect)

    # Titik ZMP (Line2D object yang bisa diupdate)
    point_plot, = ax.plot(ZMP_x, ZMP_y, 'ro')
    label = ax.text(ZMP_x, ZMP_y + 0.01 * lebar, f'ZMP ({ZMP_x:.2f}, {ZMP_y:.2f})', ha='center')

    # Atur tampilan
    ax.set_xlim(ZMP_x - panjang, ZMP_x + panjang)
    ax.set_ylim(ZMP_y - lebar, ZMP_y + lebar)
    ax.set_aspect('equal')
    ax.set_title(f'Support Polygon (luas ≈ {luas:.2f})')
    ax.grid(True)
    fig.canvas.draw()
    fig.canvas.flush_events()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # Perbarui ZMP
            com_data = node.calculate_dynamic_com()
            if not com_data:
                continue
            CoM_x, CoM_y, CoM_z = com_data
            zmp_data = node.calculate_dynamic_zmp(CoM_x, CoM_y, CoM_z)
            if not zmp_data:
                continue
            ZMP_x, ZMP_y = zmp_data

            # Update titik ZMP
            point_plot.set_data(ZMP_x, ZMP_y)
            label.set_position((ZMP_x, ZMP_y + 0.01 * lebar))
            label.set_text(f'ZMP ({ZMP_x:.2f}, {ZMP_y:.2f})')

            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Dihentikan oleh pengguna.")

    finally:
        plt.ioff()
        plt.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    plot_dynamic_zmp_area()