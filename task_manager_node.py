#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import math
import time

# ── Behaviour Tree Node States ───────────────────────────────
SUCCESS = 'SUCCESS'
FAILURE = 'FAILURE'
RUNNING = 'RUNNING'

# ══════════════════════════════════════════════════════════════
# BASE CLASSES
# ══════════════════════════════════════════════════════════════

class BTNode:
    def tick(self, blackboard):
        raise NotImplementedError

class SequenceNode(BTNode):
    """Runs children in order - remembers position between ticks"""
    def __init__(self, name, children):
        self.name = name
        self.children = children
        self.current_index = 0

    def tick(self, blackboard):
        node = blackboard.get('ros_node')
        while self.current_index < len(self.children):
            child = self.children[self.current_index]
            result = child.tick(blackboard)
            print(f'SEQ {self.name}: index={self.current_index} result={result}', flush=True)
            if result == FAILURE:
                self.current_index = 0
                return FAILURE
            if result == RUNNING:
                return RUNNING
            self.current_index += 1
        self.current_index = 0
        return SUCCESS

class FallbackNode(BTNode):
    """Tries children in order - remembers position between ticks"""
    def __init__(self, name, children):
        self.name = name
        self.children = children
        self.current_index = 0

    def tick(self, blackboard):
        while self.current_index < len(self.children):
            child = self.children[self.current_index]
            result = child.tick(blackboard)
            if result == SUCCESS:
                self.current_index = 0
                return SUCCESS
            if result == RUNNING:
                return RUNNING
            self.current_index += 1
        self.current_index = 0
        return FAILURE

class RepeatDecorator(BTNode):
    def __init__(self, name, child, max_attempts=999):
        self.name = name
        self.child = child
        self.attempts = 0
        self.max_attempts = max_attempts

    def tick(self, blackboard):
        if self.attempts >= self.max_attempts:
            return FAILURE
        result = self.child.tick(blackboard)
        if result == FAILURE:
            self.attempts += 1
            return RUNNING
        self.attempts = 0
        return result

class InverterDecorator(BTNode):
    def __init__(self, name, child):
        self.name = name
        self.child = child

    def tick(self, blackboard):
        result = self.child.tick(blackboard)
        if result == SUCCESS:
            return FAILURE
        if result == FAILURE:
            return SUCCESS
        return RUNNING

# ══════════════════════════════════════════════════════════════
# CONDITION NODES
# ══════════════════════════════════════════════════════════════

class IsBatteryLow(BTNode):
    def tick(self, blackboard):
        status = blackboard.get('battery_status', 'OK')
        return SUCCESS if status == 'LOW' else FAILURE

class IsCharging(BTNode):
    def tick(self, blackboard):
        status = blackboard.get('battery_status', 'OK')
        return SUCCESS if status == 'CHARGING' else FAILURE

class IsTask1Done(BTNode):
    def tick(self, blackboard):
        return SUCCESS if blackboard.get('task1_done', False) else FAILURE

class IsTask2Done(BTNode):
    def tick(self, blackboard):
        return SUCCESS if blackboard.get('task2_done', False) else FAILURE

class IsNearFire(BTNode):
    def tick(self, blackboard):
        return SUCCESS if blackboard.get('near_fire', False) else FAILURE

# ══════════════════════════════════════════════════════════════
# ACTION NODES
# ══════════════════════════════════════════════════════════════

class NavigateToColour(BTNode):
    """Searches for a colour and drives to it"""
    def __init__(self, name, colour, stop_distance=1.0):
        self.name = name
        self.colour = colour
        self.stop_distance = stop_distance
        self.state = 'IDLE'
        self.last_publish = 0

    def tick(self, blackboard):
        node = blackboard.get('ros_node')

        if self.state == 'IDLE':
            self.state = 'SEARCHING'
            self.last_publish = 0
            node.get_logger().info(f'{self.name}: Searching for {self.colour}')

        if self.state == 'IDLE':
            self.state = 'SEARCHING'
            self.last_publish = 0
            stop_cmd = String()
            stop_cmd.data = 'STOP'
            node.nav_command_pub.publish(stop_cmd)
            node.get_logger().info(f'{self.name}: Searching for {self.colour}')

        if self.state == 'SEARCHING':
            nav_status = blackboard.get('nav_status', '')
            if nav_status == 'REACHED':
                self.state = 'IDLE'
                blackboard['nav_status'] = ''
                # Stop the navigation node
                cmd = String()
                cmd.data = 'STOP'
                node.nav_command_pub.publish(cmd)
                node.get_logger().info(f'{self.name}: Goal reached!')
                return SUCCESS

            # Only republish if not yet reached
            now = time.time()
            if now - self.last_publish > 1.0:
                cmd = String()
                cmd.data = f'SEARCH:{self.colour}:{self.stop_distance}'
                node.nav_command_pub.publish(cmd)

                target = String()
                target.data = self.colour
                node.target_colour_pub.publish(target)

                self.last_publish = now

            return RUNNING

        return FAILURE

class WaitAction(BTNode):
    def __init__(self, name, wait_time):
        self.name = name
        self.wait_time = wait_time
        self.start_time = None
        self.state = 'IDLE'

    def tick(self, blackboard):
        node = blackboard.get('ros_node')
        if self.state == 'IDLE':
            self.start_time = time.time()
            self.state = 'WAITING'
            node.get_logger().info(f'{self.name}: Waiting {self.wait_time}s')
            return RUNNING
        elif self.state == 'WAITING':
            elapsed = time.time() - self.start_time
            node = blackboard.get('ros_node')
            node.get_logger().info(f'WaitAction elapsed: {elapsed:.2f}s')
            if elapsed >= self.wait_time:
                self.state = 'IDLE'
                node.get_logger().info(f'WaitAction complete!')
                return SUCCESS
            return RUNNING
        return FAILURE

class PublishSurvivorTF(BTNode):
    def __init__(self):
        self.name = 'PublishSurvivorTF'
        self.published = False

    def tick(self, blackboard):
        ros_node = blackboard.get('ros_node')
        survivor = blackboard.get('survivor_pos')
        ros_node.get_logger().info('PublishSurvivorTF: ticking!')
        pose = PoseStamped()
        pose.header.stamp = ros_node.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(survivor[0])
        pose.pose.position.y = float(survivor[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        ros_node.survivor_pose_pub.publish(pose)
        self.published = True
        ros_node.get_logger().info('Survivor pose published!')
        return SUCCESS

class ScanDam(BTNode):
    def __init__(self):
        self.name = 'ScanDam'
        self.state = 'IDLE'
        self.start_time = None

    def tick(self, blackboard):
        node = blackboard.get('ros_node')
        cmd = Twist()
        if self.state == 'IDLE':
            self.state = 'ROTATE_LEFT'
            self.start_time = time.time()
            node.get_logger().info('Scanning dam - rotating left')
            return RUNNING
        elif self.state == 'ROTATE_LEFT':
            elapsed = time.time() - self.start_time
            if elapsed < 2.0:
                cmd.angular.z = 0.35
                cmd.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd)
                return RUNNING
            else:
                self.state = 'ROTATE_RIGHT'
                self.start_time = time.time()
                node.get_logger().info('Scanning dam - rotating right')
                return RUNNING
        elif self.state == 'ROTATE_RIGHT':
            elapsed = time.time() - self.start_time
            if elapsed < 4.0:
                cmd.angular.z = -0.35
                cmd.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd)
                return RUNNING
            else:
                self.state = 'CENTRE'
                self.start_time = time.time()
                node.get_logger().info('Scanning dam - returning to centre')
                return RUNNING
        elif self.state == 'CENTRE':
            elapsed = time.time() - self.start_time
            if elapsed < 2.0:
                cmd.angular.z = 0.35
                cmd.linear.x = 0.0
                node.cmd_vel_pub.publish(cmd)
                return RUNNING
            else:
                node.cmd_vel_pub.publish(Twist())
                self.state = 'IDLE'
                node.get_logger().info('Dam scan complete!')
                return SUCCESS
        return FAILURE

class StopRobot(BTNode):
    def __init__(self, name):
        self.name = name

    def tick(self, blackboard):
        node = blackboard.get('ros_node')
        node.cmd_vel_pub.publish(Twist())
        return SUCCESS

class MarkTaskDone(BTNode):
    def __init__(self, task_key):
        self.name = f'MarkDone_{task_key}'
        self.task_key = task_key

    def tick(self, blackboard):
        blackboard[self.task_key] = True
        node = blackboard.get('ros_node')
        node.get_logger().info(f'{self.task_key} complete!')
        return SUCCESS

class MoveAwayFromFire(BTNode):
    def __init__(self):
        self.name = 'MoveAwayFromFire'
        self.state = 'IDLE'
        self.start_time = None

    def tick(self, blackboard):
        node = blackboard.get('ros_node')
        if self.state == 'IDLE':
            self.state = 'MOVING'
            self.start_time = time.time()
            node.get_logger().warn('Too close to fire! Moving away!')
            return RUNNING
        elif self.state == 'MOVING':
            elapsed = time.time() - self.start_time
            if elapsed < 2.0:
                cmd = Twist()
                cmd.linear.x = -0.3
                node.cmd_vel_pub.publish(cmd)
                return RUNNING
            else:
                node.cmd_vel_pub.publish(Twist())
                self.state = 'IDLE'
                return SUCCESS
        return FAILURE

# ══════════════════════════════════════════════════════════════
# TASK MANAGER NODE
# ══════════════════════════════════════════════════════════════

class AlwaysCheckSequence(BTNode):
    """Always checks safety nodes first, then runs mission"""
    def __init__(self, name, safety_nodes, mission_node):
        self.name = name
        self.safety_nodes = safety_nodes
        self.mission_node = mission_node

    def tick(self, blackboard):
        # Always check safety nodes first
        for safety in self.safety_nodes:
            result = safety.tick(blackboard)
            if result == FAILURE:
                return FAILURE
            if result == RUNNING:
                return RUNNING
        # Then run mission
        return self.mission_node.tick(blackboard)

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # ── Publishers ──────────────────────────────────────────
        self.goal_pub = self.create_publisher(PoseStamped, '/nav_goal', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_command_pub = self.create_publisher(String, '/nav_command', 10)
        self.target_colour_pub = self.create_publisher(String, '/target_colour', 10)

        # ── Subscribers ─────────────────────────────────────────
        self.nav_status_sub = self.create_subscription(
            String, '/nav_status', self.nav_status_callback, 10)
        self.battery_sub = self.create_subscription(
            String, '/battery_status', self.battery_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # ── TF Broadcaster ──────────────────────────────────────
        self.survivor_pose_pub = self.create_publisher(PoseStamped, '/survivor_pose', 10)

        # ── Blackboard ───────────────────────────────────────────
        self.blackboard = {
            'ros_node': self,
            'nav_status': '',
            'battery_status': 'OK',
            'task1_done': False,
            'task2_done': False,
            'near_fire': False,
            'robot_x': 0.0,
            'robot_y': 0.0,
            'survivor_pos': (15.1, 13.4),
            'medical_kit_pos': (-6.3, -16.9),
            'dam_pos': (8.7, -11.6),
            'fire_pos': (-14.2, 10.8),
            'exit_pos': (2.9, 17.2),
            'docking_pos': (24.89, 0.0),
            'survivor_goal': (14.1, 13.4),
            'medical_kit_goal': (-6.3, -15.9),
            'dam_goal': (8.2, -11.6),
            'exit_goal': (2.9, 17.2),
            'docking_goal': (23.89, 0.0),
        }

        # ── Build the Behaviour Tree ─────────────────────────────
        self.tree = self.build_tree()

        # ── Tick the tree every 0.5 seconds ─────────────────────
        self.timer = self.create_timer(0.5, self.tick_tree)
        self.get_logger().info('Task manager started!')

    def nav_status_callback(self, msg):
        self.blackboard['nav_status'] = msg.data

    def battery_callback(self, msg):
        self.blackboard['battery_status'] = msg.data

    def odom_callback(self, msg):
        self.blackboard['robot_x'] = msg.pose.pose.position.x
        self.blackboard['robot_y'] = msg.pose.pose.position.y
        rx = self.blackboard['robot_x']
        ry = self.blackboard['robot_y']
        fx, fy = self.blackboard['fire_pos']
        dist = math.sqrt((rx - fx)**2 + (ry - fy)**2)
        self.blackboard['near_fire'] = dist < 3.0

    def build_tree(self):
        task1 = SequenceNode('Task1_MedicalKit', [
            NavigateToColour('GoToSurvivor', 'green', stop_distance=0.9),
            WaitAction('WaitAtSurvivor', 1.0),
            PublishSurvivorTF(),
            NavigateToColour('GoToMedicalKit', 'yellow', stop_distance=0.9),
            NavigateToColour('ReturnToSurvivor', 'green', stop_distance=0.9),
            MarkTaskDone('task1_done'),
        ])
        task2 = SequenceNode('Task2_ScanDam', [
            NavigateToColour('GoToDam', 'blue', stop_distance=0.5),
            ScanDam(),
            MarkTaskDone('task2_done'),
        ])
        docking_sequence = SequenceNode('DockingSequence', [
            NavigateToColour('GoToDock', 'black', stop_distance=2.0),
            StopRobot('StopAtDock'),
            RepeatDecorator('WaitForCharge',
                InverterDecorator('NotCharging', IsCharging())
            ),
        ])
        battery_check = FallbackNode('BatteryCheck', [
            InverterDecorator('BatteryNotLow', IsBatteryLow()),
            docking_sequence,
        ])
        fire_safety = FallbackNode('FireSafety', [
            InverterDecorator('NotNearFire', IsNearFire()),
            MoveAwayFromFire(),
        ])
        task5 = SequenceNode('Task5_Exit', [
            NavigateToColour('GoToExit', 'purple', stop_distance=0.6),
            StopRobot('StopAtExit'),
        ])
        main_tasks = SequenceNode('MainMission', [
            task1,
            task2,
            task5,
        ])
        root = AlwaysCheckSequence('Root',
            safety_nodes=[fire_safety, battery_check],
            mission_node=main_tasks
        )
        return root

    def tick_tree(self):
        if hasattr(self, 'mission_complete') and self.mission_complete:
            return
        result = self.tree.tick(self.blackboard)
        self.get_logger().info(
            f'BT result: {result} | '
            f'nav_status: {self.blackboard["nav_status"]} | '
            f'task1: {self.blackboard["task1_done"]} | '
            f'near_fire: {self.blackboard["near_fire"]} | '
            f'battery: {self.blackboard["battery_status"]}'
        )
        if result == SUCCESS:
            self.mission_complete = True
            self.get_logger().info('Mission complete! Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
