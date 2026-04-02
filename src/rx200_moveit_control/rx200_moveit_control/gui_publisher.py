import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import messagebox
from builtin_interfaces.msg import Duration

import numpy as np
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest


class CoordinatePublisher(Node):
    def __init__(self):
        super().__init__('coordinate_publisher')

        # yell at the action client
        self.publisher = self.create_publisher(Float32MultiArray, 'goal_coordinates', 10)

        # for asking moveit a question        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        while not self.ik_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().warning('Waiting for IK service...')

    def publish_coordinates(self, x1, y1, z1, w1, x2, y2, z2, w2):
        msg = Float32MultiArray()
        msg.data = [float(x1), float(y1), float(z1), float(w1), float(x2), float(y2), float(z2), float(w2)]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published coordinates for Pt. 1: x={x1}, y={y1}, z={z1}, w={w1}")
        self.get_logger().info(f"Published coordinates for Pt. 2: x={x2}, y={y2}, z={z2}, w={w2}")

class tkinterGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.root = tk.Tk()
        self.root.title("Coordinate Sender")

        # Column Headers
        tk.Label(self.root, text="x:").grid(row=0, column=1)
        tk.Label(self.root, text="y:").grid(row=0, column=2)
        tk.Label(self.root, text="z:").grid(row=0, column=3)
        tk.Label(self.root, text="w:").grid(row=0, column=4)

        # Row 1 Label
        tk.Label(self.root, text="Pt. 1").grid(row=1, column=0)
        self.entry_x1 = tk.Entry(self.root)
        self.entry_y1 = tk.Entry(self.root)
        self.entry_z1 = tk.Entry(self.root)
        self.entry_w1 = tk.Entry(self.root)

        self.entry_x1.insert(0, "0.3")
        self.entry_y1.insert(0, "0.0")
        self.entry_z1.insert(0, "0.0")
        self.entry_w1.insert(0, "1.0")

        self.entry_x1.grid(row=1, column=1)
        self.entry_y1.grid(row=1, column=2)
        self.entry_z1.grid(row=1, column=3)
        self.entry_w1.grid(row=1, column=4)

        # Row 2 Label
        tk.Label(self.root, text="Pt. 2").grid(row=2, column=0)
        self.entry_x2 = tk.Entry(self.root)
        self.entry_y2 = tk.Entry(self.root)
        self.entry_z2 = tk.Entry(self.root)
        self.entry_w2 = tk.Entry(self.root)

        self.entry_x2.insert(0, "0.4")
        self.entry_y2.insert(0, "0.0")
        self.entry_z2.insert(0, "0.0")
        self.entry_w2.insert(0, "1.0")

        self.entry_x2.grid(row=2, column=1)
        self.entry_y2.grid(row=2, column=2)
        self.entry_z2.grid(row=2, column=3)
        self.entry_w2.grid(row=2, column=4)

        tk.Button(self.root, text="Send Coordinates", command=self.send_coordinates).grid(row=3, column=1, columnspan=2)
        tk.Button(self.root, text="Go to Standby", command=self.go_to_standby).grid(row=3, column=2, columnspan=2)
        tk.Button(self.root, text="Go to Sleep", command=self.go_to_sleep).grid(row=3, column=3, columnspan=2)

    def reachable(self, x, y, z, w=1.0):
        ik_working = False

        distance = (float(x)**2 + float(y)**2 + float(z)**2)**0.5
        if distance >= 0.55:
            return (False, 'Beyond Reach of ReactorX-200.')

        if not ik_working:
            return (True, 'IK is on vacay today.')

        ik_msg = PositionIKRequest()
        ik_msg.group_name = "interbotix_arm"
        ik_msg.pose_stamped.header.frame_id = "world"
        ik_msg.pose_stamped.pose.position.x = float(x)
        ik_msg.pose_stamped.pose.position.y = float(y)
        ik_msg.pose_stamped.pose.position.z = float(z)
        ik_msg.pose_stamped.pose.orientation.w = float(w)

        ik_msg.timeout = Duration(sec=14, nanosec=0)

        # Create the outer service request
        ik_request = GetPositionIK.Request()
        ik_request.ik_request = ik_msg

        # Call IK solver
        try:
            future = self.ros_node.ik_client.call_async(ik_request)
            rclpy.spin_until_future_complete(self.ros_node, future, timeout_sec=2.0)

            if not future.done():
                return False, "IK service timed out/Slow connection."

            response = future.result()

            if response is None:
                return False, "IK returned no response."
            elif response.error_code.val == 1:  # SUCCESS
                return True, "No error received."
            else:
                return False, f"Code: {response.error_code.val}"
        except Exception as e:
            self.ros_node.get_logger().warn(f"IK failed: {e}")
            return False, f"IK failed: {e}"

    def go_to_sleep(self):
        try:
            self.ros_node.publish_coordinates(0.12175, 0.0, 0.08, 0.1, 0.12175, 0.0, 0.08, 0.1)
        except ValueError:
            messagebox.showerror("Error", "Not Sleepy.")

    def go_to_standby(self):
        try:
            self.ros_node.publish_coordinates(0.12175, 0.0, 0.2, 0.3, 0.12175, 0.0, 0.2, 0.3)
        except ValueError:
            messagebox.showerror("Error", "Don't want to.")

    def send_coordinates(self):

        x1 = self.entry_x1.get()
        y1 = self.entry_y1.get()
        z1 = self.entry_z1.get()
        w1 = self.entry_w1.get()

        x2 = self.entry_x2.get()
        y2 = self.entry_y2.get()
        z2 = self.entry_z2.get()
        w2 = self.entry_w2.get()

        send_coords = True

        pt1_reachable, error1 = self.reachable(x1, y1, z1)
        if not pt1_reachable:
            messagebox.showerror("Error", f"Pt. 1 ({x1}, {y1}, {z1}): {error1}")
            send_coords = False

        pt2_reachable, error2 = self.reachable(x2, y2, z2)
        if not pt2_reachable:
            messagebox.showerror("Error", f"Pt. 2 ({x2}, {y2}, {z2}): {error2}")
            send_coords = False

        if send_coords:
            try:
                self.ros_node.publish_coordinates(x1, y1, z1, w1, x2, y2, z2, w2)
            except ValueError:
                messagebox.showerror("Error", "Please enter valid numbers.")
        else:
            self.ros_node.publish_coordinates(x1, y1, z1, w1, x2, y2, z2, w2)


    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    ros_node = CoordinatePublisher()

    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    gui = tkinterGUI(ros_node)
    gui.run()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
