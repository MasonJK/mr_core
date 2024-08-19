import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from builtin_interfaces.msg import Time
from mr_msgs.msg import FleetRobotPose, RobotPose, MissionData
from mr_msgs.srv import SingleGoalMission
from nav_msgs.msg import OccupancyGrid

from mr_core.core.global_path_planner import GPP
from mr_core.core.api import RobotAPI


from datetime import datetime
from enum import Enum
import math


# TODO : handle mission change

class LogLevel(Enum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3

class Robot:
    class State:
        IDLE = 'IDLE'
        ON_MISSION = 'ON_MISSION'
        DISCONNECTED = 'DISCONNECTED'

    def __init__(self):
        self.current_pose  = []
        self.last_connected = Time()
        self.mission_id = None
        self.state = Robot.State.IDLE


class Mission:
    def __init__(self):
        self.mission_data = MissionData()
        self.mission_type = ''
        self.waypoints = []


class MultiRobotCore(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').get_parameter_value().integer_value
        self.declare_parameter('disconnection_threshold', 10)
        self.disconnection_threshold = self.get_parameter('disconnection_threshold').get_parameter_value().integer_value

        self.declare_parameter('port', 8883)
        port = self.get_parameter('port').value
        self.declare_parameter('iot_endpoint', "")
        iot_endpoint = self.get_parameter('iot_endpoint').value
        self.declare_parameter('root_ca_path', "")
        root_ca_path = self.get_parameter('root_ca_path').value
        self.declare_parameter('private_key_path', "")
        private_key_path = self.get_parameter('private_key_path').value
        self.declare_parameter('cert_path', "")
        cert_path = self.get_parameter('cert_path').value

        self.api = RobotAPI(port, iot_endpoint, root_ca_path, private_key_path, cert_path, self.single_goal_mission_call)
        qos_profile = QoSProfile(depth=qos_depth)

        self.robot_fleets = {}
        self.missions = {}

        self.fleet_robot_pose_sub = self.create_subscription(
            FleetRobotPose,
            'fleet_robot_pose',
            self.fleet_robot_pose_callback,
            qos_profile
        )

        # Initialize GPP as None and subscribe to the /map topic
        self.gpp = None
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=qos_depth))

        # TODO
        self.mission_state_change_server = self.create_subscription()
        self.timer = self.create_timer(1.0, self.check_disconnected_robots)

    def stop_connection(self):
        self.api.stop_connection()

    def publish_log(self, log_level: LogLevel, log_content: str):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        if log_level == LogLevel.DEBUG:
            self.get_logger().debug(log_content)
        elif log_level == LogLevel.INFO:
            self.get_logger().info(log_content)
        elif log_level == LogLevel.WARN:
            self.get_logger().warning(log_content)
        elif log_level == LogLevel.ERROR:
            self.get_logger().error(log_content)

        self.api.publish_log(current_time, log_level.name, log_content)

    def fleet_robot_pose_callback(self, msg):
        fleet_id = msg.fleet_id

        # Check if fleet_id is already registered
        if fleet_id not in self.robot_fleets:
            self.robot_fleets[fleet_id] = {}
            self.publish_log(LogLevel.INFO, f"New fleet registered: {fleet_id}")

        for robot_pose in msg.fleet_pose:
            robot_id = robot_pose.robot_id

            # Check if robot_id is already registered under the fleet
            if robot_id not in self.robot_fleets[fleet_id]:
                # Register the new robot and log the registration
                self.robot_fleets[fleet_id][robot_id] = Robot()
                self.publish_log(LogLevel.INFO, f"New robot registered: {robot_id} in fleet {fleet_id}")

            robot = self.robot_fleets[fleet_id][robot_id]
            robot.current_pose = [robot_pose.x, robot_pose.y, robot_pose.theta]
            robot.last_connected = robot_pose.stamp

            if robot.state == Robot.State.DISCONNECTED:
                robot.state = Robot.State.IDLE
            elif robot.state == Robot.State.ON_MISSION:
                if robot.mission_id in self.missions:
                    # TODO : progress, traffic?
                    mission = self.missions[robot.mission_id]
                    # mission.mission_data.progress = self.calculate_progress(
                    #     mission.waypoints, robot_pose.utm_x, robot_pose.utm_y
                    # )
                else:
                    self.publish_log(LogLevel.ERROR, f"Robot is on unregistered mission: {robot_id} in fleet {fleet_id} with mission {robot.mission_id}")

    def map_callback(self, msg):
        if self.gpp is None:
            self.get_logger().info('Map received.')
            map_data = msg.data
            map_width = msg.info.width
            map_height = msg.info.height
            map_resolution = msg.info.resolution
            map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

            # Initialize GPP with the map data from the topic
            self.gpp = GPP(map_data, map_width, map_height, map_resolution, map_origin, self.get_logger().info)


    def single_goal_mission_call(self, mission_id: str, fleet_id: str, robot_id: str, goal_pose):
        try:
            if self.gpp is None:
                self.publish_log(LogLevel.WARN, 'GPP is not initialized. Cannot process mission.')
                return
            robot = self.robot_fleets[fleet_id][robot_id]
            if robot.state == Robot.State.ON_MISSION:
                self.publish_log(LogLevel.WARN, f'Robot {robot_id} is already on mission. Rejecting mission of {mission_id}')
                return
            # TODO : assign task to robot if robot is not listed

            # Prepare the request object
            request = SingleGoalMission.Request()
            request.mission_data.mission_id = mission_id
            request.mission_data.fleet_id = fleet_id
            request.mission_data.robot_id = robot_id
            request.mission_data.start_time = self.get_clock().now().to_msg()

            start_pose = robot.current_pose
            waypoints = self.gpp.plan_gpp(start_pose, goal_pose)
            request.waypoints = self.gpp.convert_waypoints_to_pose_stamped(waypoints)

            # Create the service client
            single_goal_mission_client = self.create_client(SingleGoalMission, 'single_goal_mission')

            # Wait for the service to be available
            if not single_goal_mission_client.wait_for_service(timeout_sec=5.0):
                self.publish_log(LogLevel.WARN, 'The single_goal_mission_client service is not available.')
                return

            # Send the request asynchronously and add a callback for when it's done
            future = single_goal_mission_client.call_async(request)
            future.add_done_callback(lambda f: self.single_goal_mission_response_callback(f, mission_id, fleet_id, robot_id, waypoints, request))

        except KeyError as e:
            self.publish_log(LogLevel.ERROR, f"Robot {robot_id} or Fleet {fleet_id} not found. Error: {str(e)}")
        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while assigning the mission: {str(e)}")

    def single_goal_mission_response_callback(self, future, mission_id, fleet_id, robot_id, waypoints, request):
        try:
            response = future.result()
            if response.result:
                self.publish_log(LogLevel.INFO, f"Mission {mission_id} successfully assigned to robot {robot_id} in fleet {fleet_id}.")

                mission = Mission()
                mission.mission_data = request.mission_data
                mission.mission_type = "SingleGoalMission"
                mission.waypoints = waypoints
                self.missions[mission_id] = mission

                robot = self.robot_fleets[fleet_id][robot_id]
                robot.state = Robot.State.ON_MISSION
                robot.mission_id = mission_id

                self.publish_log(LogLevel.INFO, f"Mission id({mission_id}) has been successfully assigned to Robot({robot_id})")
            else:
                self.publish_log(LogLevel.ERROR, f"Failed to assign mission {mission_id} to robot {robot_id} in fleet {fleet_id}.")
        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while processing the service response: {str(e)}")


    def check_disconnected_robots(self):
        current_time = self.get_clock().now()
        message = "Robot Information:\n"

        for fleet_id, robots in self.robot_fleets.items():
            for robot_id, robot in robots.items():
                time_since_last_connection = current_time - rclpy.time.Time.from_msg(robot.last_connected)
                if time_since_last_connection > Duration(seconds=self.disconnection_threshold):
                    if robot.state != Robot.State.DISCONNECTED:
                        log_content = f'Robot [{robot_id}] in fleet [{fleet_id}] is now DISCONNECTED'
                        self.publish_log(LogLevel.WARN, log_content)
                        robot.state = Robot.State.DISCONNECTED

                message += f'  Robot ID: {robot_id}\n'
                message += f'  Fleet: {fleet_id}\n'
                message += f'  Current Pose: [x: {robot.current_pose[0]}, y: {robot.current_pose[1]}, theta: {robot.current_pose[2]}]\n'
                message += f'  Last Connected: {robot.last_connected.sec} seconds, {robot.last_connected.nanosec} nanoseconds\n'
                message += f'  Mission ID: {robot.mission_id}\n'
                message += f'  State: {robot.state}\n'
                message += '-----------------------------------\n'

        self.publish_log(LogLevel.INFO, message)
