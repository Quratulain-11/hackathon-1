---
sidebar_position: 1
title: "AI-Robot Brain: Intelligent Decision Making"
---

# Module 3: AI-Robot Brain - Intelligent Decision Making

## Creating the Cognitive System

In this module, we'll develop the AI "brain" of our humanoid robot - the system that processes sensor data, makes decisions, and generates appropriate responses. This cognitive system bridges the gap between raw sensor inputs and meaningful robot actions.

## Learning Objectives

By the end of this module, you will:
- Understand the architecture of AI systems for robotics
- Implement path planning algorithms for navigation
- Create task planning systems for complex behaviors
- Build state machines and behavior trees for robot control
- Integrate AI decision-making with ROS 2 communication
- Design systems that handle uncertainty and adapt to changing conditions

## AI Architecture for Robotics

### Cognitive System Overview

The AI brain of our robot consists of several interconnected components:

1. **Perception Processing**: Interpreting sensor data to understand the environment
2. **World Modeling**: Maintaining an internal representation of the environment
3. **Planning**: Generating sequences of actions to achieve goals
4. **Control**: Executing low-level commands to actuators
5. **Learning**: Adapting behavior based on experience

### System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Perception    │    │   World Model   │    │   Task Planner  │
│   (Sensors)     │───▶│   (Knowledge)   │───▶│   (High-level)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Path Planning   │    │ Behavior Trees  │    │ Action Executor │
│ (Navigation)    │    │ (Decision Tree) │    │ (Low-level)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Path Planning Algorithms

### A* Algorithm for Navigation

A* is a popular pathfinding algorithm that balances optimality and efficiency:

```python
import heapq
import numpy as np
from typing import List, Tuple

class AStarPlanner:
    def __init__(self, occupancy_grid: np.ndarray):
        self.grid = occupancy_grid
        self.height, self.width = occupancy_grid.shape

    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Manhattan distance heuristic"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            new_x, new_y = pos[0] + dx, pos[1] + dy
            if (0 <= new_x < self.height and 0 <= new_y < self.width and
                self.grid[new_x, new_y] == 0):  # 0 = free space
                neighbors.append((new_x, new_y))
        return neighbors

    def plan_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Plan path using A* algorithm"""
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors(current):
                tentative_g = g_score[current] + 1  # Assuming uniform cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found
```

### RRT (Rapidly-exploring Random Tree) for Complex Environments

For environments with complex obstacles, RRT provides better coverage:

```python
import numpy as np
from typing import List, Tuple
import random

class RRTPlanner:
    def __init__(self, bounds: Tuple[float, float, float, float],
                 obstacle_list: List[Tuple[float, float, float]]):
        self.x_min, self.y_min, self.x_max, self.y_max = bounds
        self.obstacles = obstacle_list  # List of (x, y, radius) tuples
        self.step_size = 0.5

    def is_collision_free(self, point: Tuple[float, float]) -> bool:
        """Check if point is collision-free"""
        x, y = point
        if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
            return False

        for obs_x, obs_y, radius in self.obstacles:
            if np.sqrt((x - obs_x)**2 + (y - obs_y)**2) <= radius:
                return False
        return True

    def find_nearest_node(self, nodes: List[Tuple[float, float]],
                         target: Tuple[float, float]) -> Tuple[float, float]:
        """Find the nearest node in the tree to the target"""
        nearest = nodes[0]
        min_dist = float('inf')
        for node in nodes:
            dist = np.sqrt((node[0] - target[0])**2 + (node[1] - target[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest

    def plan_path(self, start: Tuple[float, float],
                 goal: Tuple[float, float], max_iterations: int = 1000) -> List[Tuple[float, float]]:
        """Plan path using RRT algorithm"""
        if not self.is_collision_free(start) or not self.is_collision_free(goal):
            return []

        nodes = [start]
        parent_map = {start: None}

        for _ in range(max_iterations):
            # Random sample (bias toward goal)
            if random.random() < 0.1:  # 10% chance to sample goal
                rand_point = goal
            else:
                rand_point = (random.uniform(self.x_min, self.x_max),
                             random.uniform(self.y_min, self.y_max))

            if not self.is_collision_free(rand_point):
                continue

            nearest = self.find_nearest_node(nodes, rand_point)

            # Move from nearest toward random point
            direction = np.array(rand_point) - np.array(nearest)
            direction_norm = np.linalg.norm(direction)
            if direction_norm == 0:
                continue

            if direction_norm > self.step_size:
                direction = direction * self.step_size / direction_norm

            new_point = tuple(np.array(nearest) + direction)

            if self.is_collision_free(new_point):
                nodes.append(new_point)
                parent_map[new_point] = nearest

                # Check if we're close to goal
                if np.sqrt((new_point[0] - goal[0])**2 + (new_point[1] - goal[1])**2) < self.step_size:
                    # Reconstruct path
                    path = []
                    current = new_point
                    while current is not None:
                        path.append(current)
                        current = parent_map[current]
                    return path[::-1]

        return []  # No path found
```

## Behavior Trees for Complex Actions

Behavior trees provide a structured way to create complex behaviors:

```python
from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Any

class NodeStatus(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"

class BehaviorNode(ABC):
    def __init__(self, name: str):
        self.name = name
        self.status = NodeStatus.RUNNING

    @abstractmethod
    def tick(self, blackboard: dict) -> NodeStatus:
        pass

class SequenceNode(BehaviorNode):
    def __init__(self, name: str, children: List[BehaviorNode]):
        super().__init__(name)
        self.children = children
        self.current_child_index = 0

    def tick(self, blackboard: dict) -> NodeStatus:
        while self.current_child_index < len(self.children):
            child_status = self.children[self.current_child_index].tick(blackboard)

            if child_status == NodeStatus.FAILURE:
                self.current_child_index = 0
                return NodeStatus.FAILURE
            elif child_status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif child_status == NodeStatus.SUCCESS:
                self.current_child_index += 1

        # All children succeeded
        self.current_child_index = 0
        return NodeStatus.SUCCESS

class SelectorNode(BehaviorNode):
    def __init__(self, name: str, children: List[BehaviorNode]):
        super().__init__(name)
        self.children = children
        self.current_child_index = 0

    def tick(self, blackboard: dict) -> NodeStatus:
        while self.current_child_index < len(self.children):
            child_status = self.children[self.current_child_index].tick(blackboard)

            if child_status == NodeStatus.SUCCESS:
                self.current_child_index = 0
                return NodeStatus.SUCCESS
            elif child_status == NodeStatus.RUNNING:
                return NodeStatus.RUNNING
            elif child_status == NodeStatus.FAILURE:
                self.current_child_index += 1

        # All children failed
        self.current_child_index = 0
        return NodeStatus.FAILURE

class ConditionNode(BehaviorNode):
    def __init__(self, name: str, condition_func):
        super().__init__(name)
        self.condition_func = condition_func

    def tick(self, blackboard: dict) -> NodeStatus:
        if self.condition_func(blackboard):
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE

class ActionNode(BehaviorNode):
    def __init__(self, name: str, action_func):
        super().__init__(name)
        self.action_func = action_func
        self.is_running = False

    def tick(self, blackboard: dict) -> NodeStatus:
        if not self.is_running:
            self.is_running = True
            self.action_func(blackboard)

        # Check if action is complete
        if self.is_action_complete(blackboard):
            self.is_running = False
            return NodeStatus.SUCCESS
        return NodeStatus.RUNNING

    def is_action_complete(self, blackboard: dict) -> bool:
        # Implement based on specific action
        return blackboard.get(f"{self.name}_complete", False)
```

## State Machines for Robot Control

Finite state machines provide a clear way to manage robot states:

```python
from enum import Enum
from typing import Dict, Callable

class RobotState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    MANIPULATING = "manipulating"
    WAITING = "waiting"
    EMERGENCY_STOP = "emergency_stop"

class RobotStateMachine:
    def __init__(self):
        self.current_state = RobotState.IDLE
        self.state_transitions = self._define_transitions()
        self.state_actions = self._define_state_actions()

    def _define_transitions(self) -> Dict[RobotState, Dict[str, RobotState]]:
        return {
            RobotState.IDLE: {
                'navigate_to': RobotState.NAVIGATING,
                'manipulate': RobotState.MANIPULATING,
                'wait': RobotState.WAITING,
                'emergency': RobotState.EMERGENCY_STOP
            },
            RobotState.NAVIGATING: {
                'arrived': RobotState.IDLE,
                'failed': RobotState.IDLE,
                'emergency': RobotState.EMERGENCY_STOP
            },
            RobotState.MANIPULATING: {
                'complete': RobotState.IDLE,
                'failed': RobotState.IDLE,
                'emergency': RobotState.EMERGENCY_STOP
            },
            RobotState.WAITING: {
                'continue': RobotState.IDLE,
                'emergency': RobotState.EMERGENCY_STOP
            },
            RobotState.EMERGENCY_STOP: {
                'reset': RobotState.IDLE
            }
        }

    def _define_state_actions(self) -> Dict[RobotState, Callable]:
        return {
            RobotState.IDLE: self._idle_action,
            RobotState.NAVIGATING: self._navigate_action,
            RobotState.MANIPULATING: self._manipulate_action,
            RobotState.WAITING: self._wait_action,
            RobotState.EMERGENCY_STOP: self._emergency_action
        }

    def transition(self, trigger: str) -> bool:
        """Attempt to transition to a new state based on trigger"""
        if self.current_state in self.state_transitions:
            if trigger in self.state_transitions[self.current_state]:
                new_state = self.state_transitions[self.current_state][trigger]
                old_state = self.current_state
                self.current_state = new_state

                # Execute exit action for old state and entry action for new state
                self._execute_state_change(old_state, new_state)
                return True
        return False

    def _execute_state_change(self, old_state: RobotState, new_state: RobotState):
        """Execute actions when changing states"""
        print(f"State changed from {old_state.value} to {new_state.value}")

    def _idle_action(self):
        """Action to perform in IDLE state"""
        print("Robot is idle, waiting for commands")

    def _navigate_action(self):
        """Action to perform in NAVIGATING state"""
        print("Robot is navigating to target location")

    def _manipulate_action(self):
        """Action to perform in MANIPULATING state"""
        print("Robot is performing manipulation task")

    def _wait_action(self):
        """Action to perform in WAITING state"""
        print("Robot is waiting for external event")

    def _emergency_action(self):
        """Action to perform in EMERGENCY_STOP state"""
        print("EMERGENCY: Robot stopped immediately")

    def update(self):
        """Execute the current state's action"""
        if self.current_state in self.state_actions:
            self.state_actions[self.current_state]()
```

## Integration with ROS 2

### Planning Node Implementation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import String
import numpy as np

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')

        # Publishers and subscribers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)

        # Planning algorithm
        self.planner = None
        self.current_map = None

        self.get_logger().info('Planning node initialized')

    def map_callback(self, msg):
        """Callback for map updates"""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Convert to numpy array
        grid_data = np.array(msg.data).reshape((height, width))

        # Update planner with new map
        self.current_map = grid_data
        self.planner = AStarPlanner(grid_data)

    def goal_callback(self, msg):
        """Callback for goal pose"""
        if self.planner is None:
            self.get_logger().warn('No map available for planning')
            return

        # Convert goal to grid coordinates
        goal_x = int((msg.pose.position.x - msg.info.origin.position.x) / msg.info.resolution)
        goal_y = int((msg.pose.position.y - msg.info.origin.position.y) / msg.info.resolution)

        # Convert current position to grid coordinates
        current_pos = self.get_current_position()  # Implement this method
        start_x = int((current_pos.x - msg.info.origin.position.x) / msg.info.resolution)
        start_y = int((current_pos.y - msg.info.origin.position.y) / msg.info.resolution)

        # Plan path
        path = self.planner.plan_path((start_x, start_y), (goal_x, goal_y))

        if path:
            # Convert path back to world coordinates
            world_path = self.convert_path_to_world(path, msg.info.origin, msg.info.resolution)

            # Publish path
            self.publish_path(world_path)
        else:
            self.get_logger().warn('No valid path found to goal')

    def convert_path_to_world(self, grid_path, origin, resolution):
        """Convert grid path to world coordinates"""
        world_path = Path()
        world_path.header.frame_id = 'map'

        for grid_x, grid_y in grid_path:
            point = PoseStamped()
            point.pose.position.x = origin.position.x + grid_x * resolution
            point.pose.position.y = origin.position.y + grid_y * resolution
            point.pose.position.z = origin.position.z

            world_path.poses.append(point)

        return world_path

    def publish_path(self, path):
        """Publish the planned path"""
        self.path_pub.publish(path)

    def get_current_position(self):
        """Get current robot position - implement with TF or odometry"""
        # This would typically use TF2 or odometry data
        pass

def main(args=None):
    rclpy.init(args=args)
    planning_node = PlanningNode()

    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Uncertainty and Adaptation

### Probabilistic Reasoning

```python
import numpy as np
from scipy.stats import norm

class UncertaintyHandler:
    def __init__(self):
        self.sensor_noise = 0.05  # 5cm sensor noise
        self.process_noise = 0.1  # Process model uncertainty
        self.belief_state = np.array([0, 0, 0, 0])  # x, y, vx, vy
        self.covariance = np.eye(4) * 0.1  # Initial uncertainty

    def predict(self, dt: float, control_input: np.ndarray):
        """Predict next state based on motion model"""
        # Simple motion model: x = x + vx*dt, y = y + vy*dt
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Process noise
        Q = np.eye(4) * self.process_noise

        # Predict state and covariance
        self.belief_state = F @ self.belief_state
        self.covariance = F @ self.covariance @ F.T + Q

    def update(self, measurement: np.ndarray):
        """Update belief based on sensor measurement"""
        # Measurement model (only x, y are observed)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Measurement noise
        R = np.eye(2) * self.sensor_noise

        # Kalman gain
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Innovation
        y = measurement - H @ self.belief_state

        # Update state and covariance
        self.belief_state = self.belief_state + K @ y
        self.covariance = (np.eye(4) - K @ H) @ self.covariance

    def get_confidence_ellipse(self, confidence=0.95):
        """Get confidence ellipse for 2D position"""
        # Extract position covariance
        pos_cov = self.covariance[:2, :2]

        # Eigenvalue decomposition
        eigenvals, eigenvecs = np.linalg.eigh(pos_cov)

        # Calculate ellipse parameters
        angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])
        width = 2 * np.sqrt(eigenvals[0] * chi2.ppf(confidence, 2))
        height = 2 * np.sqrt(eigenvals[1] * chi2.ppf(confidence, 2))

        return self.belief_state[0], self.belief_state[1], width, height, angle
```

## Task Planning with PDDL

For complex task planning, we can use PDDL (Planning Domain Definition Language):

```python
class PDDLPlanner:
    def __init__(self):
        self.domain = self._create_domain()
        self.problem = self._create_problem()

    def _create_domain(self):
        """Create PDDL domain for robot actions"""
        domain = """
(define (domain robot_domain)
  (:requirements :strips :typing :equality)
  (:types robot location object)
  (:predicates
    (at ?r - robot ?loc - location)
    (connected ?l1 - location ?l2 - location)
    (holding ?r - robot ?obj - object)
    (at-obj ?obj - object ?loc - location)
  )
  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (connected ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )
  (:action pickup
    :parameters (?r - robot ?obj - object ?loc - location)
    :precondition (and (at ?r ?loc) (at-obj ?obj ?loc))
    :effect (and (holding ?r ?obj) (not (at-obj ?obj ?loc)))
  )
  (:action place
    :parameters (?r - robot ?obj - object ?loc - location)
    :precondition (and (at ?r ?loc) (holding ?r ?obj))
    :effect (and (at-obj ?obj ?loc) (not (holding ?r ?obj)))
  )
)
        """
        return domain

    def _create_problem(self):
        """Create PDDL problem instance"""
        problem = """
(define (problem robot_task)
  (:domain robot_domain)
  (:objects
    robot1 - robot
    loc1 loc2 loc3 - location
    obj1 obj2 - object
  )
  (:init
    (at robot1 loc1)
    (connected loc1 loc2)
    (connected loc2 loc3)
    (connected loc3 loc1)
    (at-obj obj1 loc2)
    (at-obj obj2 loc3)
  )
  (:goal
    (and (at robot1 loc3) (holding robot1 obj1))
  )
)
        """
        return problem

    def plan(self):
        """Plan using external PDDL planner (e.g., Fast-Downward)"""
        # This would interface with an external PDDL planner
        # For now, return a placeholder plan
        return [
            ("move", "robot1", "loc1", "loc2"),
            ("pickup", "robot1", "obj1", "loc2"),
            ("move", "robot1", "loc2", "loc3")
        ]
```

## Summary

The AI brain of our robot combines multiple planning and decision-making approaches to handle complex tasks. By integrating path planning, behavior trees, state machines, and uncertainty handling, we create a robust cognitive system that can adapt to changing conditions and execute complex behaviors. In the next module, we'll explore how to integrate vision and language processing to create a complete Vision-Language-Action pipeline.