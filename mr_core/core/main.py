import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from mr_core.core.core import MultiRobotCore

def main(args=None):
    rclpy.init(args=args)
    try:
        # mqtt_connection_builder = MQTTConnectionBuilder('mqtt_connection_builder')
        core = MultiRobotCore('mr_core')
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(core)
        try:
            executor.spin()
        except KeyboardInterrupt:
            core.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            core.stop_connection()
            executor.shutdown()
            core.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
