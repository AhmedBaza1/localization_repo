import rclpy
from .tools import*


def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    imu_vis = IMUO3dVis()
    rclpy.spin(imu_vis)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_vis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()