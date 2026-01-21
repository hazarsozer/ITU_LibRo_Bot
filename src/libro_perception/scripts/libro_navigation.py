#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import math
import sys
import select
import termios
import tty
from ultralytics import YOLO

# CONFIGURATION
HUMAN_DETECTED = False
CURRENT_LANDMARK = None
CURRENT_MARKER_ID = None
SAFETY_DISABLED_UNTIL = 0 
INVESTIGATING = False 
LOST_MODE = False 

# MAP COORDINATES
LOCATIONS = {
    "start":       {"x": -8.74, "y": 2.83, "yaw": 0.0},
    "aisle_1":     {"x": -1.07, "y": 0.53, "yaw": 1.57},
    "aisle_2":     {"x":  0.98, "y": 0.53, "yaw": 1.57},
    "aisle_3":     {"x":  2.97, "y": 0.51, "yaw": 1.57},
    "aisle_4":     {"x":  4.97, "y": 0.54, "yaw": 1.57},
    "study_1":     {"x": -2.37, "y": -6.10, "yaw": -1.57},
    "study_2":     {"x":  7.41, "y":  5.61, "yaw": 0.0},
    "post_box":    {"x":  7.48, "y": -5.24, "yaw": 0.0},
    "cafe":        {"x": -6.57, "y": -0.64, "yaw": -1.57}
}

ARUCO_MAP = {
    0: "aisle_1", 1: "aisle_2", 2: "aisle_3", 3: "aisle_4",
    4: "study_1", 5: "study_2", 6: "post_box", 7: "cafe", 8: "start"
}

ARUCO_DICT_TYPE = cv2.aruco.DICT_5X5_100

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def is_data(): return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def timed_input(prompt, timeout=15):
    print(prompt, end='', flush=True)
    start_time = time.time()
    user_input = ""
    while True:
        if is_data():
            char = sys.stdin.read(1)
            if char == '\n': print(); return user_input
            user_input += char
        if time.time() - start_time > timeout:
            print("\n⏰ Idle Timeout!"); return None
        time.sleep(0.01)

class CameraMonitor(Node):
    def __init__(self):
        super().__init__('camera_monitor')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_pose = None
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt") 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
        self.aruco_params = cv2.aruco.DetectorParameters()
        print("✅ Vision Systems Ready")

    def pose_callback(self, msg): self.current_pose = msg.pose.pose

    def camera_callback(self, msg):
        global HUMAN_DETECTED, CURRENT_LANDMARK, CURRENT_MARKER_ID, INVESTIGATING, LOST_MODE
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = frame.shape
            center_x = width // 2
            
            # SAFETY (YOLO)
            results = self.model(frame, verbose=False, stream=True)
            person_found = False
            for result in results:
                for box in result.boxes:
                    if int(box.cls[0]) == 0 and (box.xyxy[0][2] - box.xyxy[0][0]) > 150:
                        person_found = True
                        cv2.rectangle(frame, (int(box.xyxy[0][0]), int(box.xyxy[0][1])), (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (0,0,255), 3)
            HUMAN_DETECTED = person_found

            # ARUCO SCANNING
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            current_marker = None
            current_id = None
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    mid = ids[i][0]
                    if mid in ARUCO_MAP:
                        target_key = ARUCO_MAP[mid]
                        current_marker = target_key
                        current_id = mid
                        perimeter = cv2.arcLength(corners[i], True)

                        # BEHAVIOR TREE
                        
                        # NORMAL MODE
                        if not LOST_MODE:
                            # We just print "Verified" to show we see it.
                            cv2.putText(frame, f"ID:{mid} (Tracking)", (10, 50 + (i*30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # LOST MODE
                        else:
                            # Center Calculation for Investigation
                            c = corners[i][0]
                            marker_center_x = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                            error_x = center_x - marker_center_x 

                            # CASE A
                            if perimeter > 200:
                                if INVESTIGATING:
                                    print(f"🛑 STOPPING INVESTIGATION: Locked on {mid}.")
                                    stop_msg = Twist(); self.cmd_vel_pub.publish(stop_msg)
                                    INVESTIGATING = False
                                
                                self.snap_to_marker(target_key) 
                                cv2.putText(frame, f"ID:{mid} FIXED!", (10, 50 + (i*30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            # CASE B
                            elif perimeter > 60:
                                if not INVESTIGATING:
                                    print(f"🧐 INVESTIGATING ID {mid}...")
                                    INVESTIGATING = True
                                
                                twist = Twist()
                                twist.linear.x = 0.15 
                                twist.angular.z = 0.005 * error_x 
                                self.cmd_vel_pub.publish(twist)
                                cv2.putText(frame, "INVESTIGATING...", (10, 50 + (i*30)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            elif INVESTIGATING:
                print("❌ LOST SIGHT. Stopping.")
                stop_msg = Twist(); self.cmd_vel_pub.publish(stop_msg)
                INVESTIGATING = False

            CURRENT_LANDMARK = current_marker
            CURRENT_MARKER_ID = current_id
            cv2.imshow("Robot Eyes", frame)
            cv2.waitKey(1)

        except Exception as e: self.get_logger().error(f"Camera Error: {e}")

    def snap_to_marker(self, marker_key):
        global LOST_MODE
        loc = LOCATIONS[marker_key]
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = loc["x"]; msg.pose.pose.position.y = loc["y"]
        
        # Enforce Yaw from Dictionary
        q = get_quaternion_from_euler(0, 0, loc["yaw"])
        msg.pose.pose.orientation.z = q[2]; msg.pose.pose.orientation.w = q[3]
        
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.01; msg.pose.covariance[7] = 0.01; msg.pose.covariance[35] = 0.01 
        
        print(f"✅ POSITION FIXED to {marker_key}. Resuming Normal Ops.")
        self.init_pose_pub.publish(msg)
        LOST_MODE = False

def main():
    rclpy.init(); camera_node = CameraMonitor(); navigator = BasicNavigator()

    q = get_quaternion_from_euler(0, 0, LOCATIONS["start"]["yaw"])
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'; init_pose.header.stamp = navigator.get_clock().now().to_msg()
    init_pose.pose.position.x = LOCATIONS["start"]["x"]; init_pose.pose.position.y = LOCATIONS["start"]["y"]
    init_pose.pose.orientation.z = q[2]; init_pose.pose.orientation.w = q[3]
    navigator.setInitialPose(init_pose); navigator.waitUntilNav2Active()

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            global LOST_MODE, INVESTIGATING
            
            if INVESTIGATING:
                if not navigator.isTaskComplete(): navigator.cancelTask()
                rclpy.spin_once(camera_node, timeout_sec=0.1)
                continue
            
            # MANUAL LOST MODE LOOP
            if LOST_MODE:
                print("⚠️ LOST MODE ACTIVE! Scanning...")
                navigator.spin(spin_dist=6.28, time_allowance=10)
                while LOST_MODE and not INVESTIGATING:
                    rclpy.spin_once(camera_node, timeout_sec=0.1)
                    if not navigator.isTaskComplete(): pass
                    else:
                        print("❌ Scan finished. No markers found.")
                        LOST_MODE = False
                        break
                continue

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n--- ITU LibRo Command Center ---")
            print("1-4: Aisles | 5-6: Study | 7: Post | 8: Cafe | 9: Entrance | L: Force Lost Mode")
            
            # SILENT AUTO-ENTRANCE CHECK
            if CURRENT_MARKER_ID != 8:
                if camera_node.current_pose:
                    dist_to_start = math.sqrt((camera_node.current_pose.position.x - LOCATIONS["start"]["x"])**2 + (camera_node.current_pose.position.y - LOCATIONS["start"]["y"])**2)
                    if dist_to_start < 1.0: pass # Silent

            choice = timed_input("Enter command: ", timeout=15)
            tty.setcbreak(sys.stdin.fileno())

            target_id = ""
            if choice == 'l' or choice == 'L':
                print("User Triggered LOST MODE.")
                LOST_MODE = True
                continue
            elif choice is None: target_id = "start"
            elif choice == '1': target_id = "aisle_1"
            elif choice == '2': target_id = "aisle_2"
            elif choice == '3': target_id = "aisle_3"
            elif choice == '4': target_id = "aisle_4"
            elif choice == '5': target_id = "study_1"
            elif choice == '6': target_id = "study_2"
            elif choice == '7': target_id = "post_box"
            elif choice == '8': target_id = "cafe"
            elif choice == '9': target_id = "start"
            elif choice == '0': break
            else: continue

            goal = PoseStamped(); goal.header.frame_id = 'map'; goal.header.stamp = navigator.get_clock().now().to_msg()
            loc = LOCATIONS[target_id]; goal.pose.position.x = loc["x"]; goal.pose.position.y = loc["y"]
            q = get_quaternion_from_euler(0, 0, loc["yaw"]); goal.pose.orientation.z = q[2]; goal.pose.orientation.w = q[3]
            print(f"🚀 Going to {target_id}..."); navigator.goToPose(goal)

            global SAFETY_DISABLED_UNTIL
            while not navigator.isTaskComplete():
                if INVESTIGATING or LOST_MODE:
                    print("⚡ NAV2 OVERRIDDEN BY LOST MODE!"); navigator.cancelTask(); break

                rclpy.spin_once(camera_node, timeout_sec=0.1)
                if is_data(): sys.stdin.read(1); print("\n🛑 INTERRUPTED!"); navigator.cancelTask(); break
                if HUMAN_DETECTED and time.time() > SAFETY_DISABLED_UNTIL:
                    print("⚠️ SAFETY STOP! (Waiting 10s...)"); navigator.cancelTask(); wait_start = time.time(); reroute_triggered = False
                    while HUMAN_DETECTED:
                        rclpy.spin_once(camera_node, timeout_sec=0.1)
                        if time.time() - wait_start > 10.0: print("🔄 REROUTING..."); SAFETY_DISABLED_UNTIL = time.time() + 5.0; reroute_triggered = True; break
                        if is_data(): sys.stdin.read(1); print("\n🛑 INTERRUPTED!"); break
                        time.sleep(0.1)
                    if reroute_triggered or not HUMAN_DETECTED: goal.header.stamp = navigator.get_clock().now().to_msg(); navigator.goToPose(goal)
                    else: break

            if navigator.getResult() == TaskResult.SUCCEEDED and not LOST_MODE:
                 rclpy.spin_once(camera_node, timeout_sec=0.1)
                 if CURRENT_LANDMARK and CURRENT_LANDMARK == target_id: print(f"✅ CONFIRMED: {CURRENT_LANDMARK}")
                 else:
                     print("🏁 Arrived. Scanning...")
                     navigator.spin(spin_dist=6.28, time_allowance=10)
                     while not navigator.isTaskComplete():
                         rclpy.spin_once(camera_node, timeout_sec=0.1)
                         if CURRENT_LANDMARK and CURRENT_LANDMARK == target_id: print(f"✅ FOUND: {CURRENT_LANDMARK}"); navigator.cancelTask(); break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        navigator.lifecycleShutdown(); camera_node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()

if __name__ == '__main__': main()
