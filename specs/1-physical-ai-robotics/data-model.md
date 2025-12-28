# Data Model: Physical AI & Humanoid Robotics

## Book Content Data Model

### Chapter/Module Structure
```
Module {
  id: string (e.g., "module-1-ros-nervous-system")
  title: string
  description: string
  learning_outcomes: string[]
  prerequisites: string[]
  content: ContentSection[]
  examples: Example[]
  exercises: Exercise[]
  duration: number (estimated minutes)
  difficulty: "beginner" | "intermediate" | "advanced"
  created_at: timestamp
  updated_at: timestamp
}

ContentSection {
  id: string
  title: string
  content: string (Markdown)
  diagrams: Diagram[]
  code_snippets: CodeSnippet[]
  type: "theory" | "practical" | "exercise" | "example"
  order: number
}
```

### Example and Code Structure
```
Example {
  id: string
  title: string
  description: string
  module_id: string
  difficulty: "beginner" | "intermediate" | "advanced"
  estimated_time: number (minutes)
  files: ExampleFile[]
  dependencies: string[] (ROS packages, Python libraries, etc.)
  setup_instructions: string
  execution_instructions: string
  expected_output: string
  troubleshooting: string[]
}

ExampleFile {
  id: string
  name: string
  path: string
  content: string
  file_type: "python" | "launch" | "config" | "urdf" | "world" | "markdown" | "other"
  is_executable: boolean
}
```

## Simulation Data Model

### Robot Model Structure
```
RobotModel {
  id: string
  name: string
  urdf_path: string
  sdf_path: string
  joint_count: number
  link_count: number
  degrees_of_freedom: number
  sensors: Sensor[]
  actuators: Actuator[]
  mass: number (kg)
  dimensions: Dimensions
  base_frame: string
  end_effector_frames: string[]
  kinematic_chain: string[]
}

Dimensions {
  length: number (m)
  width: number (m)
  height: number (m)
}

Sensor {
  id: string
  name: string
  type: "camera" | "lidar" | "imu" | "force_torque" | "gps" | "other"
  frame_id: string
  parameters: object (sensor-specific configuration)
  topic: string
  data_type: string (ROS message type)
}

Actuator {
  id: string
  name: string
  joint_name: string
  type: "position" | "velocity" | "effort"
  limits: JointLimits
  control_topic: string
  feedback_topic: string
}

JointLimits {
  min_position: number
  max_position: number
  max_velocity: number
  max_effort: number
}
```

### Simulation World Structure
```
SimulationWorld {
  id: string
  name: string
  description: string
  world_file_path: string
  objects: WorldObject[]
  lighting: LightingConfig
  physics_config: PhysicsConfig
  initial_robot_pose: Pose
  environment_variables: object
}

WorldObject {
  id: string
  name: string
  model_type: "static" | "dynamic" | "interactive"
  pose: Pose
  model_path: string
  properties: object (mass, friction, etc.)
  interactable: boolean
}

Pose {
  position: Point3D
  orientation: Quaternion
}

Point3D {
  x: number
  y: number
  z: number
}

Quaternion {
  x: number
  y: number
  z: number
  w: number
}

PhysicsConfig {
  gravity: Point3D
  solver_type: string
  time_step: number
  max_step_size: number
  real_time_factor: number
}
```

## AI and RAG Data Model

### RAG Knowledge Base Structure
```
RagDocument {
  id: string
  title: string
  content: string (Markdown)
  module_id: string
  section_id: string
  page_reference: string (e.g., "module-2/section-3")
  embedding: float[] (vector embedding)
  metadata: object (source, author, date, etc.)
  created_at: timestamp
  updated_at: timestamp
  tags: string[]
  citations: Citation[]
}

Citation {
  id: string
  source: string (book section, external reference, etc.)
  page_number: number
  paragraph_number: number
  text_snippet: string
}

RagQuery {
  id: string
  query_text: string
  query_embedding: float[]
  timestamp: timestamp
  user_id: string (optional)
  context: string (conversation history)
  retrieved_documents: RagDocument[]
  response: string
  citations: Citation[]
  confidence_score: number
  hallucination_detected: boolean
}

RagResponse {
  id: string
  query_id: string
  response_text: string
  source_documents: RagDocument[]
  citations: Citation[]
  confidence_score: number
  generated_at: timestamp
  feedback_score: number (1-5 rating from user)
}
```

### AI Interaction Structure
```
AiCommand {
  id: string
  raw_input: string (user's original command)
  parsed_intent: string
  entities: Entity[]
  confidence: number
  robot_action: RobotAction
  generated_at: timestamp
  execution_result: ExecutionResult
}

Entity {
  type: string (e.g., "object", "location", "action")
  value: string
  confidence: number
}

RobotAction {
  id: string
  type: "navigation" | "manipulation" | "perception" | "communication"
  parameters: object (action-specific parameters)
  target_frame: string
  execution_plan: string[] (sequence of sub-actions)
  estimated_duration: number
}

ExecutionResult {
  success: boolean
  error_message: string
  execution_time: number
  feedback: string
  metrics: object (success rates, etc.)
}
```

## ROS 2 Message Mapping

### Custom Message Types for Book Examples
```
VoiceCommand {
  command_text: string
  confidence: float
  timestamp: time
  speaker_id: string
}

RobotState {
  joint_positions: float[]
  joint_velocities: float[]
  joint_efforts: float[]
  base_pose: Pose
  base_twist: Twist
  timestamp: time
}

PerceptionResult {
  objects: DetectedObject[]
  image: sensor_msgs/Image
  depth: sensor_msgs/Image
  timestamp: time
}

DetectedObject {
  name: string
  class_id: int
  confidence: float
  bounding_box: BoundingBox
  pose: Pose
  dimensions: Point3D
}

BoundingBox {
  x_min: int
  y_min: int
  x_max: int
  y_max: int
}
```

## Relationships

### Content Relationships
- Module -(1:M)- ContentSection
- Module -(1:M)- Example
- ContentSection -(1:M)- CodeSnippet/Diagram
- Example -(1:M)- ExampleFile

### Simulation Relationships
- SimulationWorld -(1:M)- WorldObject
- RobotModel -(1:M)- Sensor
- RobotModel -(1:M)- Actuator
- WorldObject -(M:1)- SimulationWorld

### AI/RAG Relationships
- RagDocument -(M:1)- Module
- RagQuery -(1:1)- RagResponse
- RagResponse -(M:M)- RagDocument (via citations)
- AiCommand -(1:1)- RobotAction -(1:1)- ExecutionResult