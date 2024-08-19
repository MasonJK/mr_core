# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# RobotClientAPI.py
#
# @author Emima Jiva. (c) ai2-UPV Todos los derechos reservados.
#
# Proyecto "WASHCARROB"
# Version: 1.0
# Rev: 2023. cambios introducidos

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from mr_adapter.utils import euler_from_quaternion, quaternion_from_euler
import json

import time
from rclpy.time import Time
import logging
import threading

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# receive mission, and give back mission feedback and mission result
class RobotAPI:
    def __init__(self, port: int, iot_endpoint: str, root_ca_path: str, private_key_path: str, cert_path: str, single_goal_call_func):
        self.connected = False
        self.single_goal_call_func = single_goal_call_func
        client_id = "core"

        # MQTT Connection
        self.client = self.connect_mqtt(client_id, port, iot_endpoint, root_ca_path, private_key_path, cert_path)

        # Start a thread to keep the client running
        self.thread = threading.Thread(target=self.keep_running)
        self.thread.start()


    def stop_connection(self):
        # self.logger.info("Disconnecting from AWS IoT Core")
        self.client.disconnect()
        self.thread.join
        # self.logger.info("Disconnected")


    def connect_mqtt(self, client_id, port, iot_endpoint, root_ca_path, private_key_path, cert_path):
        client = AWSIoTMQTTClient(client_id)
        client.configureEndpoint(iot_endpoint, port)
        client.configureCredentials(root_ca_path, private_key_path, cert_path)
        client.configureAutoReconnectBackoffTime(1, 32, 20)
        client.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        client.configureDrainingFrequency(2)  # Draining: 2 Hz
        client.configureConnectDisconnectTimeout(10)  # 10 sec
        client.configureMQTTOperationTimeout(5)  # 5 sec
        try:
            client.connect()
            self.connected = True
            logger.info("[RMF] Connected to AWS IoT Core")
        except Exception as e:
            logger.error(f"[RMF] Failed to connect to AWS IoT Core: {str(e)}")
            exit(1)  # Exit the script with a non-zero status

        # Subscribe to topics
        client.subscribe('core/single_goal_mission', 1, self.on_single_goal_mission_call)

        return client

    def keep_running(self):
        while True:
            time.sleep(1)


    def on_single_goal_mission_call(self, client, userdata, msg):
        # Assuming pose will come with PoseStamped
        decoded_message=str(msg.payload.decode("utf-8"))
        mission_id=json.loads(decoded_message)['payload']['mission_id']
        fleet_id=json.loads(decoded_message)['payload']['fleet_id']
        robot_id=json.loads(decoded_message)['payload']['robot_id']
        goal_pose = [float(json.loads(decoded_message)['payload']['goal_pose_x']),
                     float(json.loads(decoded_message)['payload']['goal_pose_y']),
                     float(json.loads(decoded_message)['payload']['goal_pose_theta'])]
        # call single goal mission
        logger.info(f"RECEIVED MISSION OF {mission_id}")
        self.single_goal_call_func(mission_id, fleet_id, robot_id, goal_pose)


    def publish_log(self, stamp: str, log_level: str, log_content: str):
        # logger.info(f"{stamp}: [{log_level}] {log_content}")
        try:
            if not self.connected:
                logger.error("MQTT client is not connected. Unable to publish log.")
                return False

            data = {
                "stamp": stamp,
                "log_level": log_level,
                "log_content": log_content
            }
            self.client.publish("core/log", json.dumps(data), 0)
            return True
        except Exception as e:
            logger.error(f"Failed to publish log with exception: {e}")
            return False
