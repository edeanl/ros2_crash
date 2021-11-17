import rclpy

from turtle_trajectory_generator.trajectory_generator_class import PolyTrajectory


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PolyTrajectory()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
