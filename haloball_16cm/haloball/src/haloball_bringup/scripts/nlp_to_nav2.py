#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf_transformations
import math
import json
import re
from openai import OpenAI
from pynput import keyboard
import threading
import speech_recognition as sr

class NLPToNav2(Node):
    # change use_voice_input to false for keyboard input
    def __init__(self, use_voice_input=False):
        super().__init__('nlp_to_nav2')

        self.use_voice_input = use_voice_input
        self.tf_buffer = Buffer()  # Buffer for transforms
        self.tf_listener = TransformListener(self.tf_buffer, self)  # Listener to update transforms
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')  # Navigation action client

        # OpenAI client initialization with local URL and API key (modify to your server's ip)
        self.client = OpenAI(base_url="http://192.168.31.150:1234/v1", api_key="lm-studio")

        self.current_pose = None  # Current robot pose (x, y, yaw)
        self.pose_ready = False   # Flag if pose is valid
        self.running_command = False  # Flag if a command is currently being executed
        self.action_sequence = []  # List of parsed navigation actions
        self.current_action_index = 0  # Index of current action in the sequence

        # Known static locations with coordinates and orientation (yaw)
        self.known_locations = {
            "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "kitchen": {"x": 1.0, "y": 1.2, "yaw": 1.57},
            "charging station": {"x": -1.0, "y": 1.2, "yaw": 3.14}
        }

        # Timer to periodically update pose and check for input
        self.timer = self.create_timer(1.0, self.update_pose_and_check_input)

        if self.use_voice_input:
            self.recognizer = sr.Recognizer()  # Speech recognition engine
            self.mic = sr.Microphone()  # Microphone input
            self.ptt_listening = False  # Push-to-talk listening flag
            self.start_ptt_listener()  # Start keyboard listener for push-to-talk

    def start_ptt_listener(self):
        # Start a keyboard listener thread to detect push-to-talk key (Alt)
        print("push-to-talk key (Alt)")
        def on_press(key):
            if key == keyboard.Key.alt and not self.ptt_listening:
                self.ptt_listening = True
                # Start a thread to listen once for speech input
                threading.Thread(target=self.listen_once, daemon=True).start()
        keyboard.Listener(on_press=on_press).start()

    def listen_once(self):
        # Listen for a single voice command and process it
        print("[PTT] Listening...")
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source)  # Adjust for noise
            try:
                audio = self.recognizer.listen(source, timeout=20)  # Listen with timeout
                command = self.recognizer.recognize_google(audio)  # Recognize speech using Google API
                print(f"[PTT] Heard: {command}")
                self.running_command = True
                self.process_command(command)  # Process recognized text command
            except sr.UnknownValueError:
                print("[PTT] Could not understand audio.")
            except sr.RequestError as e:
                print(f"[PTT] Request error: {e}")
            finally:
                self.ptt_listening = False  # Reset listening flag

    def update_pose_and_check_input(self):
        # Update robot pose using TF and check for user input if not currently running a command
        try:
            now = rclpy.time.Time()
            # Lookup transform from 'map' to 'base_link' to get current pose
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            # Convert quaternion to yaw angle
            yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.current_pose = (x, y, yaw)
            self.pose_ready = True
        except Exception:
            # If transform unavailable, pose not ready
            self.pose_ready = False
            return

        if not self.running_command:
            if self.use_voice_input:
                pass  # Voice commands handled by push-to-talk listener
            else:
                # Keyboard input mode: prompt user for goal instruction
                try:
                    print("Enter goal instruction: ", end="", flush=True)
                    text = input().strip()
                    if text:
                        self.running_command = True
                        self.process_command(text)
                except EOFError:
                    rclpy.shutdown()

    def is_pose_close(self, pose1, pose2, pos_tol=0.1, yaw_tol=0.1):
        # Check if two poses are close within position and yaw tolerances
        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]
        # Normalize yaw difference to [-pi, pi]
        dyaw = abs((pose1[2] - pose2[2] + math.pi) % (2 * math.pi) - math.pi)
        return (dx*dx + dy*dy) < (pos_tol * pos_tol) and dyaw < yaw_tol

    def send_goal_direct(self, goal_pose):
        # Send a navigation goal directly to the Nav2 action server
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.wait_for_server()  # Wait for action server availability
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_sent_callback)  # Register callback for goal sent

    def goal_sent_callback(self, future):
        # Callback once goal is accepted or rejected by the server
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[Goal] Rejected.")
            self.running_command = False
            return

        print("[Goal] Accepted. Waiting for result...")
        # Wait for result and set callback for completion
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # Callback once goal navigation is complete
        print("[Goal] Complete.")
        self.current_action_index += 1  # Move to next action in sequence
        self.execute_next_action()

    def process_command(self, text):
        # Send user's navigation instruction to the LLM to get JSON action array
        response = self.client.chat.completions.create(
            model="qwen/qwen3-8b",
            messages=[
                {"role": "system", "content": """
                Convert user instructions into a JSON array of navigation actions.
                Supported formats:
                - Move: {"move": "forward", "distance": 1.0}
                - Rotate: {"rotate": "left", "angle": 90}
                - Go to known location: {"goto": "kitchen"}

                Always return a JSON array only. no_think
                """ },
                {"role": "user", "content": text}
            ],
            temperature=0.2
        )

        raw = response.choices[0].message.content.strip()
        print(f"[LLM] Output: {raw}")

        try:
            # Extract JSON array from LLM output using regex and parse it
            match = re.search(r'\[.*\]', raw, re.DOTALL)
            self.action_sequence = json.loads(match.group(0))
            self.current_action_index = 0
            self.execute_next_action()  # Start executing parsed actions
        except Exception as e:
            print(f"[ERROR] Failed to parse LLM output: {e}")
            self.running_command = False

    def execute_next_action(self):
        # Execute the next action in the current action sequence
        if self.current_action_index >= len(self.action_sequence):
            print("[Sequence] All actions completed.")
            self.running_command = False
            return

        action = self.action_sequence[self.current_action_index]

        if "goto" in action:
            # Handle 'goto' known location action
            name = action["goto"].lower().replace("_", " ")
            pose = self.known_locations.get(name)
            if pose is None:
                print(f"[Goto] Unknown location: {name}")
                self.running_command = False
                return

            # If already close to the location, skip this action
            if self.current_pose and self.is_pose_close(self.current_pose, (pose['x'], pose['y'], pose['yaw'])):
                print(f"[Goto] Already at {name}, skipping.")
                self.current_action_index += 1
                self.execute_next_action()
                return

            # Prepare PoseStamped goal for navigation
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose['x']
            goal_pose.pose.position.y = pose['y']
            q = tf_transformations.quaternion_from_euler(0, 0, pose['yaw'])
            goal_pose.pose.orientation.x = q[0]
            goal_pose.pose.orientation.y = q[1]
            goal_pose.pose.orientation.z = q[2]
            goal_pose.pose.orientation.w = q[3]

            self.send_goal_direct(goal_pose)
            return

        # For move or rotate actions, current pose must be known
        if not self.current_pose:
            print("[Init] Current pose not available.")
            self.running_command = False
            return

        x, y, yaw = self.current_pose
        if "move" in action:
            # Compute new pose after move
            goal_pose = self.compute_goal_pose(x, y, yaw, action["distance"], action["move"])
        elif "rotate" in action:
            # Compute new pose after rotation
            goal_pose = self.compute_rotation_pose(x, y, yaw, action["angle"], action["rotate"])
        else:
            print("[Action] Invalid format.")
            self.running_command = False
            return

        # Send the computed goal pose as a navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_sent_callback)

    def compute_goal_pose(self, x, y, yaw, distance, direction):
        # Calculate the target pose after moving forward or backward
        offset = distance if direction == 'forward' else -distance
        goal_x = x + offset * math.cos(yaw)
        goal_y = y + offset * math.sin(yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)  # Orientation unchanged
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        return goal_pose

    def compute_rotation_pose(self, x, y, yaw, angle, direction):
        # Calculate the target pose after rotating left or right by a given angle
        angle_rad = math.radians(angle)
        new_yaw = yaw + angle_rad if direction == 'left' else yaw - angle_rad

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x  # Position unchanged
        goal_pose.pose.position.y = y
        q = tf_transformations.quaternion_from_euler(0, 0, new_yaw)  # Updated orientation
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        return goal_pose

def main():
    rclpy.init()  # Initialize ROS 2 Python client library
    node = NLPToNav2()  # Create node instance
    rclpy.spin(node)  # Spin to process callbacks
    rclpy.shutdown()  # Shutdown ROS 2 on exit

if __name__ == '__main__':
    main()
