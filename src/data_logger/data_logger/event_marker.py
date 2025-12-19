import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HELP = """
Event marker keyboard:
0 = none
f = push_front
b = push_back
l = push_left
r = push_right
d = push_down
u = lift_up
q = quit
"""

KEY_TO_EVENT = {
    '0': 'none',
    'f': 'push_front',
    'b': 'push_back',
    'l': 'push_left',
    'r': 'push_right',
    'd': 'push_down',
    'u': 'lift_up'
}

class EventMarker(Node):
    def __init__(self):
        super().__init__('event_marker')
        self.pub = self.create_publisher(String, '/experiment/event', 10)
        self.get_logger().info(HELP)

    def publish_event(self, ev: str):
        msg = String()
        msg.data = ev
        self.pub.publish(msg)
        self.get_logger().info(f"EVENT -> {ev}")

def main(args=None):
    rclpy.init(args=args)
    node = EventMarker()

    try:
        while rclpy.ok():
            key = input("key> ").strip().lower()
            if key == 'q':
                break
            ev = KEY_TO_EVENT.get(key)
            if ev is None:
                print("Unknown key. " + HELP)
                continue
            node.publish_event(ev)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()