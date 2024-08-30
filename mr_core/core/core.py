import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

from builtin_interfaces.msg import Time
from mr_msgs.msg import FleetRobotPose, RobotPose, MissionData
from mr_msgs.srv import MissionGoal, MissionStateChange, MissionCancel
from nav_msgs.msg import OccupancyGrid

from mr_core.core.global_path_planner import GPP
from mr_core.core.api import RobotAPI


from datetime import datetime
from enum import Enum
import math


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
        FAILED = 'FAILED'

    def __init__(self):
        self.current_pose  = []
        self.last_connected = Time()
        self.mission_id = None
        self.state = Robot.State.IDLE


class Mission:
    def __init__(self):
        self.mission_data = MissionData()
        self.goals = []
        self.mission_uuids = []
        self.waypoints = []
        self.segment_distances = []
        self.current_goal_idx = 0
        self.total_distance = 0.0
        self.distance_covered = 0.0
        self.progress = 0.0


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

        self.api = RobotAPI(port, iot_endpoint, root_ca_path, private_key_path, cert_path,
                            self.mission_goal_call,
                            self.mission_cancel_call)
        qos_profile = QoSProfile(depth=qos_depth)

        self.robot_fleets = {}
        self.missions = {}

        self.fleet_robot_pose_sub = self.create_subscription(
            FleetRobotPose,
            'fleet_robot_pose',
            self.fleet_robot_pose_callback,
            qos_profile
        )
        self.mission_state_change_server = self.create_service(MissionStateChange, "mission_state_change",
                                                               self.mission_state_change_callback)
        self.timer = self.create_timer(1.0, self.check_disconnected_robots)

        # Initialize GPP as None and subscribe to the /map topic
        self.gpp = None
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=qos_depth))

    def stop_connection(self):
        self.api.stop_connection()

    def publish_log(self, log_level: LogLevel, log_content: str):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        log_message = f"({self.get_name()}) - {log_content}"

        if log_level == LogLevel.DEBUG:
            self.get_logger().debug(log_message)
        elif log_level == LogLevel.INFO:
            self.get_logger().info(log_message)
        elif log_level == LogLevel.WARN:
            self.get_logger().warning(log_message)
        elif log_level == LogLevel.ERROR:
            self.get_logger().error(log_message)

        self.api.publish_log(current_time, log_level.name, log_message)

    def fleet_robot_pose_callback(self, msg):
        fleet_id = msg.fleet_id

        # Check if fleet_id is already registered
        if fleet_id not in self.robot_fleets:
            self.robot_fleets[fleet_id] = {}
            self.publish_log(LogLevel.INFO, f"Registered new fleet({fleet_id})")

        for robot_pose in msg.fleet_pose:
            robot_id = robot_pose.robot_id

            # Check if robot_id is already registered under the fleet
            if robot_id not in self.robot_fleets[fleet_id]:
                # Register the new robot and log the registration
                self.robot_fleets[fleet_id][robot_id] = Robot()
                self.publish_log(LogLevel.INFO, f"Registered new robot({fleet_id} - {robot_id})")

            robot = self.robot_fleets[fleet_id][robot_id]
            robot.current_pose = [robot_pose.x, robot_pose.y, robot_pose.theta]
            robot.last_connected = robot_pose.stamp

            # TODO : policy of failed robot
            if robot.state == Robot.State.FAILED:
                pass
            elif robot.state == Robot.State.DISCONNECTED:
                robot.state = Robot.State.IDLE
            elif robot.state == Robot.State.ON_MISSION:
                # TODO: need to solve
                if robot.mission_id in self.missions:
                    # TODO : traffic?
                    mission = self.missions[robot.mission_id]
                    mission.progress = self.calculate_progress(robot.mission_id, robot_pose)
                else:
                    self.publish_log(LogLevel.ERROR, f"Robot({fleet_id} - {robot_id}) is on unregistered mission({robot.mission_id})")


    def map_callback(self, msg):
        if self.gpp is None:
            self.publish_log(LogLevel.INFO, 'Map received.')
            map_data = msg.data
            map_width = msg.info.width
            map_height = msg.info.height
            map_resolution = msg.info.resolution
            map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

            # Initialize GPP with the map data from the topic
            self.gpp = GPP(map_data, map_width, map_height, map_resolution, map_origin, self.get_logger().info)


    def check_disconnected_robots(self):
        current_time = self.get_clock().now()

        for fleet_id, robots in self.robot_fleets.items():
            for robot_id, robot in robots.items():
                time_since_last_connection = current_time - rclpy.time.Time.from_msg(robot.last_connected)
                if time_since_last_connection > Duration(seconds=self.disconnection_threshold):
                    if robot.state != Robot.State.DISCONNECTED:
                        log_content = f'Robot ({fleet_id} - {robot_id}) is not responding for {self.disconnection_threshold}. Now going to DISCONNECTED state'
                        self.publish_log(LogLevel.WARN, log_content)
                        robot.state = Robot.State.DISCONNECTED

        # for debugging
        message = '++++++++++++++++++++++++++++++++++++++\n'
        message += f"{current_time}\nRobot Information:\n"
        for fleet_id, robots in self.robot_fleets.items():
            for robot_id, robot in robots.items():
                message += f'  Robot ID: {robot_id}\n'
                message += f'  Fleet: {fleet_id}\n'
                message += f'  Current Pose: [x: {robot.current_pose[0]}, y: {robot.current_pose[1]}, theta: {robot.current_pose[2]}]\n'
                message += f'  Last Connected: {robot.last_connected.sec} seconds, {robot.last_connected.nanosec} nanoseconds\n'
                message += f'  Mission ID: {robot.mission_id}\n'
                message += f'  State: {robot.state}\n'
                message += '-----------------------------------\n'

        message += "Mission Information:\n"
        for id, mission in self.missions.items():
            message += f'  Mission ID: {mission.mission_data.mission_id}\n'
            message += f'  Assigned Robot: {mission.mission_data.fleet_id}/{mission.mission_data.robot_id}\n'
            message += f'  Current Mission UUID: {mission.mission_uuids[mission.current_goal_idx]}\n'
            # message += f'  Mission Type: {mission.mission_type}\n'
            message += f'  Mission Progress: {mission.progress}'
            message += '\n++++++++++++++++++++++++++++++++++++++\n'

        self.publish_log(LogLevel.INFO, message)


    def mission_state_change_callback(self, request, response):
        mission_id = None
        for id, mission in self.missions.items():
            if request.mission_uuid in mission.mission_uuids:
                mission_id = id
                break

        if mission_id == None:
            self.publish_log(LogLevel.ERROR, f"Mission ID not found for UUID({request.mission_uuid})")
            response.result = False
            return response

        mission = self.missions[mission_id]
        fleet_id = mission.mission_data.fleet_id
        robot_id = mission.mission_data.robot_id
        robot = self.robot_fleets[fleet_id][robot_id]

        if request.mission_state == MissionStateChange.Request.STATUS_ACCEPTED:
            # Log the execution state
            self.publish_log(LogLevel.INFO, f"Mission({mission_id}) successfully assigned to robot({fleet_id} - {robot_id}). Now Executing...")
            robot.state = Robot.State.ON_MISSION
            robot.mission_id = mission_id

        elif request.mission_state == MissionStateChange.Request.STATUS_REJECTED:
            self.publish_log(LogLevel.ERROR, f"Mission({mission_id}) goal rejected by robot({fleet_id} - {robot_id})")
            del self.missions[mission_id]
            robot.state = Robot.State.IDLE
            robot.mission_id = None

        elif request.mission_state == MissionStateChange.Request.STATUS_SUCCEEDED:
            current_goal_idx = self.missions[mission_id].current_goal_idx

            if current_goal_idx < len(self.missions[mission_id].goals)-1:
                self.publish_log(LogLevel.INFO, f"Mission({mission_id}) succeeded navigating to goal waypoint(idx - {current_goal_idx}). Navigating to next goal..")

                # calculate gpp
                start_pose = robot.current_pose
                waypoints = self.gpp.plan_gpp(start_pose, mission.goals[current_goal_idx+1])
                mission.waypoints = waypoints

                mission.segment_distances, mission.total_distance = self.calculate_segment_distances(waypoints)

                self.send_goal(mission.mission_uuids[current_goal_idx+1], mission.mission_data, waypoints)
                self.missions[mission_id].current_goal_idx += 1
            else:
                self.publish_log(LogLevel.INFO, f"Mission({mission_id}) succeeded. Robot({fleet_id} - {robot_id}) going back to IDLE state")
                del self.missions[mission_id]
                robot.state = Robot.State.IDLE
                robot.mission_id = None

        elif request.mission_state == MissionStateChange.Request.STATUS_CANCELED:
            self.publish_log(LogLevel.WARN, f"Mission({mission_id}) was canceled. Robot({fleet_id} - {robot_id}) going back to IDLE state")
            del self.missions[mission_id]
            robot.state = Robot.State.IDLE
            robot.mission_id = None

        # TODO : better fail message
        elif request.mission_state == MissionStateChange.Request.STATUS_ABORTED:
            # Log the abortion, delete the mission, and set the robot state to IDLE
            self.publish_log(LogLevel.ERROR, f"Mission({mission_id}) was aborted.")
            del self.missions[mission_id]
            robot.state = Robot.State.FAILED
            robot.mission_id = None
        else:
            self.publish_log(LogLevel.WARN, f"Unknown mission state received for mission({mission_id})")
            response.result = False
            return response

        response.result = True
        return response


    def mission_goal_call(self, mission_id: str, fleet_id: str, robot_id: str, goal_poses):
        try:
            if self.gpp is None:
                self.publish_log(LogLevel.WARN, 'GPP is not initialized. Cannot process mission.')
                return

            # TODO : assign task to robot if robot is not listed
            robot = self.robot_fleets[fleet_id][robot_id]
            if robot.state == Robot.State.FAILED:
                self.publish_log(LogLevel.ERROR, f'Robot({fleet_id} - {robot_id}) currently unable to take mission. Rejecting mission({mission_id})')
                return
            elif robot.state == Robot.State.ON_MISSION:
                self.publish_log(LogLevel.WARN, f'Robot({fleet_id} - {robot_id}) is already on mission. Rejecting mission({mission_id})')
                return

            mission_uuids = []
            for idx in range(len(goal_poses)):
                mission_uuids.append(mission_id+"_"+str(idx))

            # calculate gpp
            start_pose = robot.current_pose
            waypoints = self.gpp.plan_gpp(start_pose, goal_poses[0])

            # mission data declaration
            mission_data = MissionData()
            mission_data.mission_id = mission_id
            mission_data.fleet_id = fleet_id
            mission_data.robot_id = robot_id

            # mission declaration
            mission = Mission()
            mission.mission_data = mission_data
            mission.goals = goal_poses
            mission.mission_uuids = mission_uuids
            mission.waypoints = waypoints
            mission.segment_distances, mission.total_distance = self.calculate_segment_distances(waypoints)
            mission.current_goal_idx = 0
            mission.distance_covered = 0.0

            self.missions[mission_id] = mission

            self.send_goal(mission_uuids[0], mission_data, waypoints)

        except KeyError as e:
            self.publish_log(LogLevel.ERROR, f"Robot({robot_id}) or Fleet({fleet_id}) not found. Error: {str(e)}")
        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while assigning the mission: {str(e)}")


    def send_goal(self, mission_uuid, mission_data, waypoints_):
        try:
            # TODO temp
            waypoints = [waypoints_[-1]]

            request = MissionGoal.Request()
            request.mission_data = mission_data
            request.mission_data.start_time = self.get_clock().now().to_msg()
            request.mission_uuid = mission_uuid
            request.waypoints = self.gpp.convert_waypoints_to_pose_stamped(waypoints)

            service_name = "/" + mission_data.fleet_id + "/mission_goal"
            mission_goal_client = self.create_client(MissionGoal, service_name)

            # Wait for the service to be available
            if not mission_goal_client.wait_for_service(timeout_sec=5.0):
                self.publish_log(LogLevel.WARN, 'The mission_goal_client service is not available.')
                return

            # Send the request asynchronously and add a callback for when it's done
            future = mission_goal_client.call_async(request)
            future.add_done_callback(lambda f: self.mission_goal_response_callback(f, mission_data.mission_id, mission_data.fleet_id, mission_data.robot_id, request))

        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while sending the mission: {str(e)}")


    def mission_goal_response_callback(self, future, mission_id, fleet_id, robot_id, request):
        try:
            response = future.result()
            if response.result:
                self.publish_log(LogLevel.INFO, f"Successfully sent mission({mission_id}) to robot({fleet_id} - {robot_id})")
            else:
                self.publish_log(LogLevel.ERROR, f"Failed to assign mission({mission_id}) to robot({fleet_id} - {robot_id}). Discarding mission..")
                del self.missions[mission_id]
        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while processing the service response: {str(e)}")


    def mission_cancel_call(self, mission_id: str, fleet_id = None, robot_id = None):
        try:
            # Check if the mission exists and is assigned to the robot
            if mission_id not in self.missions:
                self.publish_log(LogLevel.ERROR, f"Mission({mission_id}) not found.")
                return

            mission = self.missions[mission_id]

            # If fleet_id or robot_id is empty, find them from the mission data
            if not fleet_id or not robot_id:
                fleet_id = mission.mission_data.fleet_id
                robot_id = mission.mission_data.robot_id

                if not fleet_id or not robot_id:
                    self.publish_log(LogLevel.ERROR, "Unable to find fleet_id or robot_id associated with the mission.")
                    return


            if mission.mission_data.fleet_id != fleet_id or mission.mission_data.robot_id != robot_id:
                self.publish_log(LogLevel.ERROR, f"Mission({mission_id}) does not match fleet({fleet_id}) or robot({robot_id})")
                return

            robot = self.robot_fleets[fleet_id][robot_id]

            # Check if the robot is currently on self mission
            if robot.state != Robot.State.ON_MISSION or robot.mission_id != mission_id:
                self.publish_log(LogLevel.WARN, f"Robot({fleet_id} - {robot_id}) is not currently executing mission({mission_id})")
                return

            # Prepare the service request
            request = MissionCancel.Request()
            request.mission_data.mission_id = mission_id
            request.mission_data.fleet_id = fleet_id
            request.mission_data.robot_id = robot_id
            request.mission_uuid = mission.mission_uuids[mission.current_goal_idx]

            # Create the client and wait for the service to be available
            service_name = "/" + request.mission_data.fleet_id + "/mission_cancel"
            mission_cancel_client = self.create_client(MissionCancel, service_name)
            if not mission_cancel_client.wait_for_service(timeout_sec=5.0):
                self.publish_log(LogLevel.WARN, 'The mission_cancel service is not available.')
                return

            # Send the request asynchronously and add a done callback
            future = mission_cancel_client.call_async(request)
            future.add_done_callback(lambda f: self.publish_log(LogLevel.INFO,
                                                                f"Sending cancellation for mission({mission_id}) result : {future.result().result}"))

        except KeyError as e:
            self.publish_log(LogLevel.ERROR, f"Robot({robot_id}) or Fleet({fleet_id}) not found. Error: {str(e)}")
        except Exception as e:
            self.publish_log(LogLevel.ERROR, f"An error occurred while trying to cancel the mission({mission_id}): {str(e)}")


    def calculate_segment_distances(self, waypoints):
        if not waypoints:
            self.publish_log(LogLevel.WARN, "No waypoints. Cannot calculate progress")
            return

        total_distance = 0.0
        segment_distances = []

        for i in range(1, len(waypoints)):
            segment_distance = self.calculate_distance(waypoints[i-1], waypoints[i])
            segment_distances.append(segment_distance)
            total_distance += segment_distance

        return segment_distances, total_distance

    def calculate_progress(self, mission_id, current_pose: RobotPose):
        mission = self.missions[mission_id]
        waypoints = mission.waypoints
        segment_distances = mission.segment_distances
        total_distance = mission.total_distance

        if not waypoints:
            self.publish_log(LogLevel.WARN, "No waypoints. Cannot calculate progress")
            return

        closest_index = min(range(len(waypoints)), key=lambda i: self.calculate_distance(current_pose, waypoints[i]))
        distance_covered = sum(segment_distances[:closest_index])
        self.missions[mission_id].distance_covered = distance_covered

        # Add the remaining distance from the closest waypoint to the current position
        if closest_index < len(waypoints) - 1:
            distance_covered += self.calculate_distance(waypoints[closest_index], current_pose)

        return min((distance_covered / total_distance) * 100, 100) if total_distance > 0 else 0.0


    def calculate_distance(self, pose1: RobotPose, pose2: RobotPose):
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)


        # #####################################################################################
        # # TEMP : temporary filtering waypoints

        # def filter_waypoints(waypoints, angle_threshold=0.1):
        #     if len(waypoints) < 2:
        #         return waypoints

        #     # List to store filtered important waypoints
        #     filtered_waypoints = [waypoints[0]]  # Always include the first waypoint

        #     def calculate_angle(p1, p2, p3):
        #         """Calculate the angle between the vectors p1->p2 and p2->p3."""
        #         v1 = (p2.x - p1.x, p2.y - p1.y)
        #         v2 = (p3.x - p2.x, p3.y - p2.y)
        #         dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        #         magnitude_v1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        #         magnitude_v2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
        #         if magnitude_v1 * magnitude_v2 == 0:
        #             return 0
        #         cos_angle = dot_product / (magnitude_v1 * magnitude_v2)
        #         # Clamp the cosine to avoid any possible floating-point errors
        #         cos_angle = max(-1.0, min(1.0, cos_angle))
        #         angle = math.acos(cos_angle)
        #         return angle

        #     for i in range(1, len(waypoints) - 1):
        #         angle = calculate_angle(waypoints[i - 1], waypoints[i], waypoints[i + 1])
        #         if angle > angle_threshold:
        #             filtered_waypoints.append(waypoints[i])

        #     # Always include the last waypoint as it's the goal
        #     filtered_waypoints.append(waypoints[-1])

        #     return filtered_waypoints

        # waypoints = filter_waypoints(waypoints)
        # # #####################################################################################
