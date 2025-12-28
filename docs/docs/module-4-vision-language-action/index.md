---
sidebar_position: 1
title: "Vision-Language-Action Pipeline"
---

# Module 4: Vision-Language-Action Pipeline

## Integrating Perception, Language, and Action

In this module, we'll create the complete pipeline that enables our humanoid robot to understand complex voice commands in the context of visual information and execute appropriate actions. This Vision-Language-Action (VLA) pipeline represents the cutting edge of embodied AI.

## Learning Objectives

By the end of this module, you will:
- Understand the architecture of Vision-Language-Action systems
- Implement multimodal perception combining vision and language
- Create natural language processing for robotic commands
- Build action execution systems that bridge AI decisions and ROS 2 commands
- Integrate all components into a unified pipeline
- Handle complex scenarios requiring both visual and linguistic understanding

## VLA System Architecture

### Overview of the Pipeline

The Vision-Language-Action pipeline connects three critical components:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Vision        │    │   Language      │    │   Action        │
│   Processing    │───▶│   Processing    │───▶│   Execution     │
│   (Images)      │    │   (Commands)    │    │   (Commands)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Object          │    │ Command         │    │ Task            │
│ Detection       │    │ Parsing         │    │ Planning        │
│ Recognition     │    │ & Intent        │    │ & Execution     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Vision Processing Component

### Object Detection and Recognition

We'll implement a vision system that can identify objects in the robot's field of view:

```python
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
from typing import List, Tuple, Dict

class VisionProcessor:
    def __init__(self):
        # Load pre-trained object detection model (e.g., YOLOv5, Detectron2, etc.)
        # For this example, we'll use a generic approach
        self.transforms = transforms.Compose([
            transforms.Resize((416, 416)),
            transforms.ToTensor(),
        ])

        # Initialize model (placeholder - would use actual model)
        self.model = self._load_model()

        # Common object classes
        self.object_classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def _load_model(self):
        """Load pre-trained vision model"""
        # Placeholder for actual model loading
        # In practice, you might load YOLOv5, Detectron2, or similar
        return None

    def detect_objects(self, image: np.ndarray) -> List[Dict]:
        """Detect and recognize objects in the image"""
        # Convert numpy array to PIL Image
        pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Apply transforms
        input_tensor = self.transforms(pil_image).unsqueeze(0)

        # Run inference (placeholder)
        # In practice: detections = self.model(input_tensor)
        detections = self._mock_detections(image)  # Mock implementation

        return detections

    def _mock_detections(self, image: np.ndarray) -> List[Dict]:
        """Mock detection for demonstration purposes"""
        # In a real implementation, this would use an actual model
        # For now, return mock detections
        height, width = image.shape[:2]

        mock_detections = [
            {
                'class': 'red cube',
                'confidence': 0.89,
                'bbox': [int(width*0.3), int(height*0.4), int(width*0.4), int(height*0.5)],
                'center': [int(width*0.35), int(height*0.45)]
            },
            {
                'class': 'blue sphere',
                'confidence': 0.76,
                'bbox': [int(width*0.6), int(height*0.3), int(width*0.7), int(height*0.4)],
                'center': [int(width*0.65), int(height*0.35)]
            },
            {
                'class': 'green cylinder',
                'confidence': 0.82,
                'bbox': [int(width*0.2), int(height*0.6), int(width*0.3), int(height*0.7)],
                'center': [int(width*0.25), int(height*0.65)]
            }
        ]

        return mock_detections

    def extract_spatial_relationships(self, detections: List[Dict], image_shape: Tuple) -> Dict:
        """Extract spatial relationships between detected objects"""
        height, width = image_shape[:2]
        center_x, center_y = width // 2, height // 2

        relationships = {
            'left_objects': [],
            'right_objects': [],
            'front_objects': [],
            'back_objects': [],
            'near_objects': [],
            'far_objects': []
        }

        for detection in detections:
            obj_center_x, obj_center_y = detection['center']

            # Determine horizontal position relative to image center
            if obj_center_x < center_x:
                relationships['left_objects'].append(detection)
            else:
                relationships['right_objects'].append(detection)

            # Determine vertical position (simplified depth)
            bbox_width = detection['bbox'][2] - detection['bbox'][0]
            # Approximate distance based on object size in image
            if bbox_width > width * 0.2:  # Large = near
                relationships['near_objects'].append(detection)
            else:
                relationships['far_objects'].append(detection)

        return relationships
```

### Visual Question Answering Component

```python
class VisualQuestionAnswering:
    def __init__(self):
        # Initialize VQA model (CLIP, BLIP, etc.)
        self.vqa_model = self._load_vqa_model()

    def _load_vqa_model(self):
        """Load pre-trained VQA model"""
        # Placeholder for actual VQA model
        return None

    def answer_question(self, image: np.ndarray, question: str) -> str:
        """Answer a question about the image content"""
        # In practice, this would use a multimodal model like CLIP or BLIP
        # For demonstration, return a mock answer based on object detection
        vision_processor = VisionProcessor()
        detections = vision_processor.detect_objects(image)

        # Simple keyword matching for demonstration
        question_lower = question.lower()

        if 'red' in question_lower:
            red_objects = [obj for obj in detections if 'red' in obj['class'].lower()]
            if red_objects:
                return f"I see a {red_objects[0]['class']} in the image."

        if 'left' in question_lower:
            relationships = vision_processor.extract_spatial_relationships(detections, image.shape)
            if relationships['left_objects']:
                left_obj = relationships['left_objects'][0]
                return f"To the left, I see a {left_obj['class']}."

        # Default response
        object_names = [obj['class'] for obj in detections if obj['confidence'] > 0.7]
        if object_names:
            return f"I see: {', '.join(object_names[:3])}."
        else:
            return "I don't see any clear objects in the image."
```

## Natural Language Processing Component

### Command Parsing and Intent Recognition

```python
import re
from typing import Dict, List, Tuple
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    action: str
    target_object: str
    spatial_reference: str
    direction: str
    distance: float
    confidence: float

class LanguageProcessor:
    def __init__(self):
        self.action_keywords = {
            'move': ['move', 'go', 'navigate', 'walk', 'step', 'proceed'],
            'pick': ['pick', 'grasp', 'grab', 'take', 'lift', 'collect'],
            'place': ['place', 'put', 'set', 'drop', 'release'],
            'look': ['look', 'see', 'find', 'locate', 'search'],
            'speak': ['speak', 'say', 'tell', 'announce']
        }

        self.spatial_keywords = {
            'left': ['left', 'left side', 'to the left'],
            'right': ['right', 'right side', 'to the right'],
            'front': ['front', 'in front', 'ahead'],
            'back': ['back', 'behind', 'back side'],
            'near': ['near', 'close', 'nearby', 'close to'],
            'far': ['far', 'distant', 'away']
        }

        self.distance_patterns = [
            (r'(\d+(?:\.\d+)?)\s*(meters?|m)', lambda x: float(x.group(1))),
            (r'(\d+(?:\.\d+)?)\s*(centimeters?|cm)', lambda x: float(x.group(1)) / 100.0),
            (r'(\d+(?:\.\d+)?)\s*(steps?)', lambda x: float(x.group(1)) * 0.75)  # 0.75m per step
        ]

    def parse_command(self, command: str) -> ParsedCommand:
        """Parse a natural language command into structured format"""
        command_lower = command.lower().strip()

        # Extract action
        action = self._extract_action(command_lower)

        # Extract target object
        target_object = self._extract_target_object(command_lower)

        # Extract spatial reference
        spatial_reference = self._extract_spatial_reference(command_lower)

        # Extract direction
        direction = self._extract_direction(command_lower)

        # Extract distance
        distance = self._extract_distance(command_lower)

        # Calculate confidence based on keyword matches
        confidence = self._calculate_confidence(command_lower, action, target_object)

        return ParsedCommand(
            action=action,
            target_object=target_object,
            spatial_reference=spatial_reference,
            direction=direction,
            distance=distance,
            confidence=confidence
        )

    def _extract_action(self, command: str) -> str:
        """Extract the main action from the command"""
        for action, keywords in self.action_keywords.items():
            for keyword in keywords:
                if keyword in command:
                    return action
        return "unknown"

    def _extract_target_object(self, command: str) -> str:
        """Extract target object from command"""
        # Look for color + object patterns
        color_object_pattern = r'(red|blue|green|yellow|white|black|large|small)\s+(\w+)'
        match = re.search(color_object_pattern, command)
        if match:
            return f"{match.group(1)} {match.group(2)}"

        # Look for simple object names
        object_patterns = [
            r'cube', r'sphere', r'cylinder', r'box', r'ball', r'object',
            r'cup', r'bottle', r'book', r'chair', r'table'
        ]

        for pattern in object_patterns:
            if re.search(pattern, command):
                return re.search(pattern, command).group(0)

        return "unknown"

    def _extract_spatial_reference(self, command: str) -> str:
        """Extract spatial reference (left, right, etc.)"""
        for reference, keywords in self.spatial_keywords.items():
            for keyword in keywords:
                if keyword in command:
                    return reference
        return "none"

    def _extract_direction(self, command: str) -> str:
        """Extract movement direction"""
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
        for direction in directions:
            if direction in command:
                return direction
        return "none"

    def _extract_distance(self, command: str) -> float:
        """Extract distance from command"""
        for pattern, converter in self.distance_patterns:
            match = re.search(pattern, command)
            if match:
                return converter(match)
        return 1.0  # Default distance

    def _calculate_confidence(self, command: str, action: str, target: str) -> float:
        """Calculate confidence in the parsed command"""
        confidence = 0.5  # Base confidence

        # Increase confidence if action was found
        if action != "unknown":
            confidence += 0.3

        # Increase confidence if target was found
        if target != "unknown":
            confidence += 0.2

        return min(confidence, 1.0)
```

### Semantic Understanding with Large Language Models

```python
class SemanticProcessor:
    def __init__(self):
        # In practice, this would connect to OpenAI API, HuggingFace, or similar
        self.llm_model = None

    def understand_command(self, command: str, context: Dict = None) -> Dict:
        """Use LLM to understand complex commands with context"""
        # Create a prompt for the LLM
        prompt = self._create_understanding_prompt(command, context)

        # In practice: response = self.llm_model.generate(prompt)
        # For demonstration, return a structured response
        return self._mock_semantic_analysis(command)

    def _create_understanding_prompt(self, command: str, context: Dict) -> str:
        """Create a prompt for semantic understanding"""
        prompt = f"""
        You are a semantic analyzer for robot commands. Analyze the following command:

        Command: "{command}"

        Context: {context or 'No additional context provided'}

        Please provide:
        1. The primary action to be performed
        2. The target object or location
        3. Any spatial relationships or directions
        4. The sequence of sub-actions if complex
        5. Potential ambiguities or clarifications needed

        Respond in JSON format with keys: action, target, spatial_relationships,
        sub_actions, ambiguities.
        """
        return prompt

    def _mock_semantic_analysis(self, command: str) -> Dict:
        """Mock semantic analysis for demonstration"""
        # This would be replaced with actual LLM call
        analysis = {
            "action": "pick",
            "target": "red cube",
            "spatial_relationships": ["left of robot", "2 meters away"],
            "sub_actions": [
                "navigate to object",
                "align gripper",
                "grasp object",
                "lift object"
            ],
            "ambiguities": []
        }

        # Adjust based on command
        if "blue" in command.lower():
            analysis["target"] = "blue sphere"
        elif "green" in command.lower():
            analysis["target"] = "green cylinder"

        if "left" in command.lower():
            analysis["spatial_relationships"] = ["left of robot"]

        return analysis
```

## Action Execution Component

### Task Planning and Execution

```python
from enum import Enum
from typing import List, Dict, Any

class TaskStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILED = "failed"
    CANCELLED = "cancelled"

class ActionExecutor:
    def __init__(self):
        self.current_task = None
        self.task_status = TaskStatus.PENDING
        self.ros_interface = None  # Would connect to ROS 2

    def execute_parsed_command(self, parsed_command: ParsedCommand,
                              visual_context: List[Dict] = None) -> TaskStatus:
        """Execute a parsed command with visual context"""
        print(f"Executing command: {parsed_command.action} {parsed_command.target_object}")

        if parsed_command.action == "pick":
            return self._execute_pick_command(parsed_command, visual_context)
        elif parsed_command.action == "move":
            return self._execute_move_command(parsed_command, visual_context)
        elif parsed_command.action == "place":
            return self._execute_place_command(parsed_command, visual_context)
        else:
            print(f"Unknown action: {parsed_command.action}")
            return TaskStatus.FAILED

    def _execute_pick_command(self, parsed_command: ParsedCommand,
                            visual_context: List[Dict]) -> TaskStatus:
        """Execute pick command"""
        # Find the target object in visual context
        target_obj = self._find_object_in_context(parsed_command.target_object, visual_context)

        if not target_obj:
            print(f"Could not find {parsed_command.target_object} in visual context")
            return TaskStatus.FAILED

        print(f"Found {target_obj['class']} at {target_obj['center']}")

        # Plan path to object
        navigation_result = self._navigate_to_object(target_obj)
        if navigation_result != TaskStatus.SUCCESS:
            return navigation_result

        # Execute pick action
        pick_result = self._perform_pick_action(target_obj)
        return pick_result

    def _execute_move_command(self, parsed_command: ParsedCommand,
                            visual_context: List[Dict]) -> TaskStatus:
        """Execute move command"""
        # Determine target location based on spatial reference
        target_location = self._determine_target_location(parsed_command, visual_context)

        if not target_location:
            print("Could not determine target location")
            return TaskStatus.FAILED

        # Navigate to target
        navigation_result = self._navigate_to_location(target_location, parsed_command.distance)
        return navigation_result

    def _execute_place_command(self, parsed_command: ParsedCommand,
                             visual_context: List[Dict]) -> TaskStatus:
        """Execute place command"""
        # Find placement location
        placement_location = self._determine_placement_location(parsed_command, visual_context)

        if not placement_location:
            print("Could not determine placement location")
            return TaskStatus.FAILED

        # Navigate to placement location
        navigation_result = self._navigate_to_location(placement_location, 0.5)  # Close distance
        if navigation_result != TaskStatus.SUCCESS:
            return navigation_result

        # Release object
        place_result = self._perform_place_action(placement_location)
        return place_result

    def _find_object_in_context(self, target_name: str, context: List[Dict]) -> Dict:
        """Find an object in the visual context"""
        for obj in context:
            if target_name.lower() in obj['class'].lower():
                return obj
        return None

    def _navigate_to_object(self, obj_info: Dict) -> TaskStatus:
        """Navigate the robot to the specified object"""
        print(f"Navigating to object at {obj_info['center']}")
        # In practice, this would send navigation commands via ROS 2
        # For now, simulate success
        return TaskStatus.SUCCESS

    def _perform_pick_action(self, obj_info: Dict) -> TaskStatus:
        """Perform the physical pick action"""
        print(f"Performing pick action on {obj_info['class']}")
        # In practice, this would control robot arms/grippers via ROS 2
        # For now, simulate success
        return TaskStatus.SUCCESS

    def _determine_target_location(self, parsed_command: ParsedCommand,
                                 visual_context: List[Dict]) -> Dict:
        """Determine target location based on command and visual context"""
        if parsed_command.spatial_reference == "left":
            # Find leftmost object or position
            relationships = VisionProcessor().extract_spatial_relationships(
                visual_context, (480, 640, 3))  # Assuming image shape
            if relationships['left_objects']:
                return relationships['left_objects'][0]['center']
        elif parsed_command.direction:
            # Use direction and distance
            return {
                'direction': parsed_command.direction,
                'distance': parsed_command.distance
            }

        return None

    def _navigate_to_location(self, location: Dict, distance: float) -> TaskStatus:
        """Navigate to a specific location"""
        print(f"Navigating to location: {location} with distance: {distance}")
        # In practice, send navigation commands via ROS 2
        return TaskStatus.SUCCESS

    def _determine_placement_location(self, parsed_command: ParsedCommand,
                                    visual_context: List[Dict]) -> Dict:
        """Determine where to place the object"""
        # For now, use a default placement location
        # In practice, this would consider surfaces, safety, etc.
        return {'x': 1.0, 'y': 0.0, 'z': 0.5}

    def _perform_place_action(self, location: Dict) -> TaskStatus:
        """Perform the physical place action"""
        print(f"Performing place action at {location}")
        # In practice, control robot arms/grippers via ROS 2
        return TaskStatus.SUCCESS
```

## Integration: The Complete VLA Pipeline

```python
class VisionLanguageActionPipeline:
    def __init__(self):
        self.vision_processor = VisionProcessor()
        self.language_processor = LanguageProcessor()
        self.semantic_processor = SemanticProcessor()
        self.action_executor = ActionExecutor()

    def process_command(self, image: np.ndarray, command: str) -> Dict:
        """Process a complete VLA command"""
        print(f"Processing VLA command: {command}")

        # Step 1: Process visual input
        print("Step 1: Processing visual input...")
        visual_detections = self.vision_processor.detect_objects(image)
        spatial_context = self.vision_processor.extract_spatial_relationships(
            visual_detections, image.shape)

        print(f"Detected objects: {[obj['class'] for obj in visual_detections]}")

        # Step 2: Process language command
        print("Step 2: Processing language command...")
        parsed_command = self.language_processor.parse_command(command)
        semantic_analysis = self.semantic_processor.understand_command(
            command, {
                'detected_objects': [obj['class'] for obj in visual_detections],
                'spatial_context': spatial_context
            }
        )

        print(f"Parsed command: {parsed_command.action} {parsed_command.target_object}")
        print(f"Semantic analysis: {semantic_analysis['action']}")

        # Step 3: Execute action with visual context
        print("Step 3: Executing action...")
        execution_result = self.action_executor.execute_parsed_command(
            parsed_command, visual_detections)

        return {
            'command': command,
            'parsed_command': parsed_command,
            'visual_detections': visual_detections,
            'spatial_context': spatial_context,
            'semantic_analysis': semantic_analysis,
            'execution_result': execution_result.value,
            'confidence': parsed_command.confidence
        }

# Example usage
def example_vla_pipeline():
    """Example of using the VLA pipeline"""
    pipeline = VisionLanguageActionPipeline()

    # Simulate an image (in practice, this would come from robot's camera)
    import numpy as np
    dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    # Example commands
    commands = [
        "Pick up the red cube on the left",
        "Move forward 2 meters",
        "Find the blue sphere and go near it"
    ]

    for command in commands:
        print(f"\n{'='*50}")
        print(f"Processing command: {command}")
        result = pipeline.process_command(dummy_image, command)
        print(f"Result: {result['execution_result']}")
        print(f"Confidence: {result['confidence']:.2f}")

if __name__ == "__main__":
    example_vla_pipeline()
```

## ROS 2 Integration

### VLA Node Implementation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import numpy as np

class VLAPipelineNode(Node):
    def __init__(self):
        super().__init__('vla_pipeline_node')

        # Initialize VLA pipeline
        self.vla_pipeline = VisionLanguageActionPipeline()
        self.cv_bridge = CvBridge()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, '/head_camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10)
        self.result_pub = self.create_publisher(String, '/vla_result', 10)

        # Store latest image for processing
        self.latest_image = None

        self.get_logger().info('VLA Pipeline node initialized')

    def image_callback(self, msg):
        """Callback for receiving camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def command_callback(self, msg):
        """Callback for receiving voice commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if self.latest_image is not None:
            # Process the command with the latest image
            result = self.vla_pipeline.process_command(self.latest_image, command)

            # Publish result
            result_msg = String()
            result_msg.data = f"Command '{command}' executed with status: {result['execution_result']}"
            self.result_pub.publish(result_msg)

            self.get_logger().info(f'Published result: {result_msg.data}')
        else:
            self.get_logger().warn('No image available for processing command')

def main(args=None):
    rclpy.init(args=args)
    vla_node = VLAPipelineNode()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Complex Scenarios

### Multi-step Task Execution

```python
class MultiStepTaskExecutor:
    def __init__(self):
        self.vla_pipeline = VisionLanguageActionPipeline()
        self.task_queue = []
        self.current_step = 0

    def execute_complex_command(self, image: np.ndarray, command: str) -> Dict:
        """Execute a complex command that may involve multiple steps"""
        # First, analyze the command for multiple sub-actions
        semantic_analysis = self.vla_pipeline.semantic_processor.understand_command(command)

        if len(semantic_analysis['sub_actions']) > 1:
            # Break down into sub-tasks
            self.task_queue = semantic_analysis['sub_actions']
            return self._execute_sub_tasks(image, command)
        else:
            # Execute as single task
            return self.vla_pipeline.process_command(image, command)

    def _execute_sub_tasks(self, image: np.ndarray, original_command: str) -> Dict:
        """Execute a sequence of sub-tasks"""
        results = []

        for i, sub_task in enumerate(self.task_queue):
            self.get_logger().info(f"Executing sub-task {i+1}/{len(self.task_queue)}: {sub_task}")

            # Process each sub-task
            result = self.vla_pipeline.process_command(image, sub_task)
            results.append(result)

            # Update image if needed (for dynamic scenes)
            # In practice, get fresh image from robot's sensors

            if result['execution_result'] != 'success':
                self.get_logger().warn(f"Sub-task failed: {sub_task}")
                break

        return {
            'original_command': original_command,
            'sub_task_results': results,
            'overall_success': all(r['execution_result'] == 'success' for r in results),
            'completed_tasks': len([r for r in results if r['execution_result'] == 'success'])
        }
```

## Error Handling and Recovery

### Robustness Mechanisms

```python
class RobustVLAProcessor:
    def __init__(self):
        self.vla_pipeline = VisionLanguageActionPipeline()
        self.max_retries = 3
        self.confidence_threshold = 0.7

    def process_with_retry(self, image: np.ndarray, command: str) -> Dict:
        """Process command with retry logic and confidence checking"""
        for attempt in range(self.max_retries):
            result = self.vla_pipeline.process_command(image, command)

            if result['confidence'] >= self.confidence_threshold:
                if result['execution_result'] == 'success':
                    return result
                else:
                    # Low confidence in failure - try again with modified approach
                    self.get_logger().info(f"Attempt {attempt + 1} failed, retrying...")
            else:
                self.get_logger().warn(f"Low confidence ({result['confidence']:.2f}), retrying...")

            if attempt < self.max_retries - 1:
                # In practice, might request rephrasing or additional sensing
                time.sleep(0.5)  # Brief pause before retry

        return result

    def handle_ambiguity(self, command: str, visual_context: List[Dict]) -> str:
        """Handle ambiguous commands by requesting clarification"""
        semantic_analysis = self.vla_pipeline.semantic_processor.understand_command(command)

        if semantic_analysis['ambiguities']:
            # In practice, this might trigger a dialogue system
            clarification_request = self._generate_clarification_request(
                command, semantic_analysis['ambiguities'])
            return clarification_request

        return "proceed"  # No ambiguities to resolve

    def _generate_clarification_request(self, command: str, ambiguities: List[str]) -> str:
        """Generate a clarification request for ambiguous commands"""
        # This would interface with a dialogue system
        # For now, return a simple clarification request
        return f"I'm not sure about your command '{command}'. Could you clarify: {', '.join(ambiguities)}"
```

## Summary

The Vision-Language-Action pipeline creates a complete system for embodied AI, allowing our humanoid robot to understand complex commands in visual contexts and execute appropriate actions. By integrating perception, language understanding, and action execution, we create a robot that can perform sophisticated tasks requiring both visual and linguistic intelligence. This completes our four-module journey in building an autonomous humanoid robot system.