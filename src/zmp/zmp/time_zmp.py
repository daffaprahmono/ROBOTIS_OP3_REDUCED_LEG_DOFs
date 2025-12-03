import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from my_msg.msg import Data
import time


class ZmpPlotter(Node):
    def __init__(self):
        super().__init__('zmp_plotter')

        # Subscribe ke topic "zmp"
        self.subscription = self.create_subscription(Data, 'zmp', self.listener_callback, 10)

        # Data untuk grafik
        self.zmp_x_values = []
        self.time_values = []
        self.start_time = time.time()

        # Setup matplotlib
        plt.ion()  # mode interaktif
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label="ZMP-X")
        self.ax.set_xlabel("Waktu (s)")
        self.ax.set_ylabel("ZMP-X")
        self.ax.set_title("Grafik ZMP-X terhadap Waktu")
        self.ax.legend()
        self.ax.grid(True)

    def listener_callback(self, msg: Data):
        # Hitung waktu relatif sejak start node
        current_time = time.time() - self.start_time
        zmp_x_int = int(msg.zmp_x)
        # Simpan data
        self.zmp_x_values.append(zmp_x_int)
        self.time_values.append(current_time)

        # Update plot
        self.line.set_xdata(self.time_values)
        self.line.set_ydata(self.zmp_x_values)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ZmpPlotter()
    try:
        rclpy.spin(node)  # jalanin node sampai Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()  # tampilkan grafik terakhir sebelum keluar
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()