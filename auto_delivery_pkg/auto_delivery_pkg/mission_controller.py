import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto


class MissionState(Enum):
    STARTUP = auto()
    SEARCH = auto()
    ALIGN_FRONT = auto()
    TURN_AROUND = auto()
    BACKUP_PARK = auto()
    DROP_PACKAGE = auto()
    DONE = auto()


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.state = MissionState.STARTUP

        # Publisher that sends "<node_name>:<0_or_1>" strings
        self.node_control_pub = self.create_publisher(
            String,
            'node_control',
            10
        )

        # State machine tick (for now: advance every 4 seconds for testing)
        self.timer = self.create_timer(4.0, self.state_step)

        self.get_logger().info("MissionController initialized in STARTUP state")

        # On startup, set nodes according to the guide
        self.enter_startup()

    def send_activation(self, node_name: str, value: int) -> None:
        """
        Helper to publish ON/OFF commands.
        value: 1 = activate, 0 = deactivate
        """
        msg = String()
        msg.data = f"{node_name}:{value}"
        self.node_control_pub.publish(msg)
        # This line is where we actually command other nodes to toggle.
        self.get_logger().info(f"[TOGGLE] {msg.data}")

    # ---------- State entry helpers (match the node_activation_flow guide) ----------

    def enter_startup(self):
        """
        STARTUP:
          mission_controller: 1 (this node)
          box_detection: 1
          apriltag_node: 1
          parking_controller: 0
          apriltag_back_node: 0
          servo_controller: 0
        """
        self.get_logger().info("[STATE] STARTUP → setting initial node activations")
        self.send_activation("box_detection", 1)
        self.send_activation("apriltag_node", 1)
        self.send_activation("parking_controller", 0)
        self.send_activation("apriltag_back_node", 0)
        self.send_activation("servo_controller", 0)

    def enter_search(self):
        """
        SEARCH:
          Same activations as STARTUP, but now the robot is driving around
          looking for the container (YOLO + front AprilTag).
        """
        self.get_logger().info("[STATE] SEARCH")
        # No change needed if already in this configuration, but we set explicitly for clarity.
        self.send_activation("box_detection", 1)
        self.send_activation("apriltag_node", 1)
        self.send_activation("parking_controller", 0)
        self.send_activation("apriltag_back_node", 0)
        self.send_activation("servo_controller", 0)

    def enter_align_front(self):
        """
        ALIGN_FRONT:
          mission_controller: 1
          box_detection: 0 (turn off YOLO to save compute)
          apriltag_node: 1 (use front AprilTag to align)
          parking_controller: 0
          apriltag_back_node: 0
          servo_controller: 0
        """
        self.get_logger().info("[STATE] ALIGN_FRONT")
        self.send_activation("box_detection", 0)
        self.send_activation("apriltag_node", 1)
        self.send_activation("parking_controller", 0)
        self.send_activation("apriltag_back_node", 0)
        self.send_activation("servo_controller", 0)

    def enter_turn_around(self):
        """
        TURN_AROUND:
          mission_controller: 1
          box_detection: 0
          apriltag_node: 0
          apriltag_back_node: 1 (rear AprilTag)
          parking_controller: 1 (handle 180° maneuver)
          servo_controller: 1 (ON but idle, ready for drop later)
        """
        self.get_logger().info("[STATE] TURN_AROUND")
        self.send_activation("box_detection", 0)
        self.send_activation("apriltag_node", 0)
        self.send_activation("apriltag_back_node", 1)
        self.send_activation("parking_controller", 1)
        self.send_activation("servo_controller", 1)

    def enter_backup_park(self):
        """
        BACKUP_PARK:
          mission_controller: 1
          box_detection: 0
          apriltag_node: 0
          apriltag_back_node: 1
          parking_controller: 1 (reverse towards container)
          servo_controller: 1 (still ON, idle)
        """
        self.get_logger().info("[STATE] BACKUP_PARK")
        self.send_activation("box_detection", 0)
        self.send_activation("apriltag_node", 0)
        self.send_activation("apriltag_back_node", 1)
        self.send_activation("parking_controller", 1)
        self.send_activation("servo_controller", 1)

    def enter_drop_package(self):
        """
        DROP_PACKAGE:
          mission_controller: 1
          box_detection: 0
          apriltag_node: 0
          apriltag_back_node: 1 (optional: keep ON monitoring)
          parking_controller: 0 (no more motion)
          servo_controller: 1 (actively tilting bed)
        """
        self.get_logger().info("[STATE] DROP_PACKAGE")
        self.send_activation("box_detection", 0)
        self.send_activation("apriltag_node", 0)
        self.send_activation("apriltag_back_node", 1)
        self.send_activation("parking_controller", 0)
        self.send_activation("servo_controller", 1)

    def enter_done(self):
        """
        DONE:
          mission_controller: 1
          all other nodes: 0
        """
        self.get_logger().info("[STATE] DONE")
        self.send_activation("box_detection", 0)
        self.send_activation("apriltag_node", 0)
        self.send_activation("apriltag_back_node", 0)
        self.send_activation("parking_controller", 0)
        self.send_activation("servo_controller", 0)

    # ---------- Simple demo FSM: advance through states on a timer ----------

    def state_step(self) -> None:
        """
        For now this just auto-steps through the mission phases so you can
        test all node toggles on your Mac without hardware.

        Later you'll replace these transitions with real conditions
        (e.g., "tag found", "aligned", "turn complete", "backed up", "drop done").
        """
        self.get_logger().info(f"[FSM] Current state: {self.state.name}")

        if self.state == MissionState.STARTUP:
            self.get_logger().info("Transition: STARTUP → SEARCH")
            self.state = MissionState.SEARCH
            self.enter_search()

        elif self.state == MissionState.SEARCH:
            self.get_logger().info("Transition: SEARCH → ALIGN_FRONT (pretend container found)")
            self.state = MissionState.ALIGN_FRONT
            self.enter_align_front()

        elif self.state == MissionState.ALIGN_FRONT:
            self.get_logger().info("Transition: ALIGN_FRONT → TURN_AROUND (pretend aligned)")
            self.state = MissionState.TURN_AROUND
            self.enter_turn_around()

        elif self.state == MissionState.TURN_AROUND:
            self.get_logger().info("Transition: TURN_AROUND → BACKUP_PARK (pretend 180° complete)")
            self.state = MissionState.BACKUP_PARK
            self.enter_backup_park()

        elif self.state == MissionState.BACKUP_PARK:
            self.get_logger().info("Transition: BACKUP_PARK → DROP_PACKAGE (pretend at drop position)")
            self.state = MissionState.DROP_PACKAGE
            self.enter_drop_package()

        elif self.state == MissionState.DROP_PACKAGE:
            self.get_logger().info("Transition: DROP_PACKAGE → DONE (pretend drop complete)")
            self.state = MissionState.DONE
            self.enter_done()

        elif self.state == MissionState.DONE:
            self.get_logger().info("FSM is in DONE state; no further transitions.")
            # self.timer.cancel()  # optional: stop ticking
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()