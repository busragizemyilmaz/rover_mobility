#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class TankDriveJoystick(Node):

    def __init__(self):
        super().__init__('tank_drive_joystick')

        # -------------------------
        # Declare Parameters
        # -------------------------
        self.declare_parameter("deadzone", 0.1)
        self.declare_parameter("max_acceleration", 1.5)   # units per second
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("control_rate", 20.0)      # Hz
        self.declare_parameter("left_axis_index", 1)
        self.declare_parameter("right_axis_index", 4)
        self.declare_parameter("mode_button_index", 0)    # PID/PWM Mode Button (X button (Xbox=2, PS4=0))

        # Get Parameters
        self.deadzone = self.get_parameter("deadzone").value
        self.max_acceleration = self.get_parameter("max_acceleration").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.control_rate = self.get_parameter("control_rate").value
        self.left_axis_index = self.get_parameter("left_axis_index").value
        self.right_axis_index = self.get_parameter("right_axis_index").value
        self.mode_button_index = self.get_parameter("mode_button_index").value

        self.control_period = 1.0 / self.control_rate

        # -------------------------
        # State Variables
        # -------------------------
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0

        self.target_left_speed = 0.0
        self.target_right_speed = 0.0

        self.control_mode = 0  # 0: PWM, 1: PID
        self.last_button_state = 0

        self.last_joy_time = self.get_clock().now()
        self.timeout_active = False

        # -------------------------
        # ROS Interfaces
        # -------------------------
        self.joy_sub = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            10
        )

        self.wheel_pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_speeds",
            10
        )

        self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info("Tank Drive Joystick Node Started. Mode: PWM (0)")

    # --------------------------------------------------
    # Utility Functions
    # --------------------------------------------------

    def apply_deadzone(self, value: float) -> float:
        """Ignore small joystick noise."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def limit_acceleration(self, target: float, current: float, dt: float) -> float:
        """
        Rate limiter:
        Limits how fast speed can change per second.
        """
        error = target - current
        max_delta = self.max_acceleration * dt

        if error > max_delta:
            delta = max_delta
        elif error < -max_delta:
            delta = -max_delta
        else:
            delta = error

        new_speed = current + delta

        # Clamp to [-1, 1]
        return max(min(new_speed, 1.0), -1.0)

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def joy_callback(self, msg: Joy):

        self.last_joy_time = self.get_clock().now()

        if self.timeout_active:
            self.get_logger().info("Joystick connection restored.")
            self.timeout_active = False

        # --- Axis inputs ---
        try:
            left_input = msg.axes[self.left_axis_index]
            right_input = msg.axes[self.right_axis_index]
        except IndexError:
            self.get_logger().error("Joystick axis index out of range!")
            return

        self.target_left_speed = self.apply_deadzone(left_input)
        self.target_right_speed = self.apply_deadzone(right_input)

        # --- Mode toggle button (rising edge only) ---
        try:
            current_mode_button = msg.buttons[self.mode_button_index]
        except IndexError:
            self.get_logger().error("Mode button index out of range!")
            return

        if current_mode_button == 1 and self.last_button_state == 0:
            # Rising edge detected → toggle mode
            self.control_mode = 1 if self.control_mode == 0 else 0

            mode_name = "PID (1)" if self.control_mode == 1 else "PWM (0)"
            self.get_logger().info(f"Control mode switched to: {mode_name}")

        self.last_button_state = current_mode_button

    def control_loop(self):

        dt = self.control_period  # Fixed timestep (deterministic)

        current_time = self.get_clock().now()
        time_since_joy = (current_time - self.last_joy_time).nanoseconds / 1e9

        # Safety timeout
        if time_since_joy > self.timeout_sec:
            if not self.timeout_active:
                self.get_logger().warn("Joystick timeout. Stopping rover.")
                self.timeout_active = True

            self.target_left_speed = 0.0
            self.target_right_speed = 0.0

        # Apply acceleration limiting
        self.current_left_speed = self.limit_acceleration(
            self.target_left_speed,
            self.current_left_speed,
            dt
        )

        self.current_right_speed = self.limit_acceleration(
            self.target_right_speed,
            self.current_right_speed,
            dt
        )

        # Publish wheel speeds and control mode combined in a 5-element array
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [
            float(self.control_mode),          # Control Mode (0.0=PWM, 1.0=PID)
            float(self.current_left_speed),    # Front Left
            float(self.current_left_speed),    # Rear Left
            float(self.current_right_speed),   # Front Right
            float(self.current_right_speed)    # Rear Right
        ]
        self.wheel_pub.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TankDriveJoystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()