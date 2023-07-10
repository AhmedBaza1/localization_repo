import rclpy
from .tools import*


def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    frame_saver = Frame_saver()
    rclpy.spin(frame_saver)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frame_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()