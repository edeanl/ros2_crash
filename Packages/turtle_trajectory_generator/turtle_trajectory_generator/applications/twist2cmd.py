import rclpy

from turtle_trajectory_generator.trajectory_generator_class import Twists2CMD


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Twists2CMD()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
