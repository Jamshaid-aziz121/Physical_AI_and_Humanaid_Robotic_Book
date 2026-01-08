# Data Model for Physical AI & Humanoid Robotics Book

## Entity: RobotModel
- **name**: String - Unique identifier for the robot model
- **description**: Text - Human-readable description of the robot
- **urdf_path**: String - Path to the URDF file for the robot model
- **kinematics**: Object - Kinematic properties of the robot
  - **joints**: Array - List of joint definitions
  - **links**: Array - List of link definitions
  - **base_frame**: String - Base coordinate frame for the robot
- **sensors**: Array - List of sensors attached to the robot
  - **type**: Enum (lidar, camera, imu, etc.)
  - **position**: Object - Position of the sensor on the robot
  - **properties**: Object - Specific properties for the sensor type

## Entity: SimulationEnvironment
- **name**: String - Unique identifier for the environment
- **description**: Text - Human-readable description of the environment
- **sdf_path**: String - Path to the SDF file for the environment
- **objects**: Array - List of static/dynamic objects in the environment
- **physics_properties**: Object - Physics parameters for the simulation
  - **gravity**: Vector3 - Gravity vector (x, y, z)
  - **friction**: Float - Default friction coefficient
  - **damping**: Float - Default damping coefficient
- **lighting**: Object - Lighting configuration for the environment

## Entity: LearningModule
- **id**: String - Unique identifier for the module
- **title**: String - Display title for the module
- **description**: Text - Detailed description of the module content
- **duration_weeks**: Integer - Estimated duration to complete the module
- **prerequisites**: Array - List of prerequisite modules or concepts
- **learning_outcomes**: Array - List of skills/knowledge to be acquired
- **content_paths**: Object - Paths to various content types
  - **theory**: String - Path to theoretical content
  - **code_examples**: String - Path to code examples directory
  - **exercises**: String - Path to exercises directory
  - **solutions**: String - Path to solutions directory

## Entity: CodeExample
- **id**: String - Unique identifier for the example
- **title**: String - Brief title of the example
- **description**: Text - Explanation of what the example demonstrates
- **language**: String - Programming language used
- **dependencies**: Array - List of required dependencies
- **files**: Array - List of file paths comprising the example
- **execution_context**: Object - Information about how to run the example
  - **environment**: String - Target environment (simulator, real robot, etc.)
  - **setup_commands**: Array - Commands to set up the environment
  - **run_command**: String - Command to execute the example

## Entity: Exercise
- **id**: String - Unique identifier for the exercise
- **title**: String - Brief title of the exercise
- **description**: Text - Detailed description of the exercise
- **difficulty**: Enum (beginner, intermediate, advanced)
- **estimated_time_minutes**: Integer - Expected time to complete
- **requirements**: Array - List of requirements for the exercise
- **validation_criteria**: Array - List of criteria to validate completion
- **solution_reference**: String - Path to the solution reference
- **simulation_scenario**: String - Optional path to simulation scenario for the exercise

## Entity: UserProgress
- **user_id**: String - Identifier for the user
- **module_id**: String - Identifier for the module being progressed through
- **completed_exercises**: Array - List of completed exercise IDs
- **last_accessed**: DateTime - Timestamp of last access
- **completion_percentage**: Float - Percentage of module completed
- **notes**: Text - Optional user notes about their progress

## Entity: TaskPlan
- **id**: String - Unique identifier for the task plan
- **name**: String - Name of the task plan
- **description**: Text - Description of what the task plan accomplishes
- **steps**: Array - Ordered list of steps to complete the task
  - **step_number**: Integer - Order of the step
  - **action**: String - Action to be performed
  - **parameters**: Object - Parameters for the action
  - **expected_result**: String - What should happen when the step is completed
- **robot_model**: String - Robot model the task plan is designed for
- **environment**: String - Environment where the task should be executed

## Entity: PerceptionData
- **id**: String - Unique identifier for the perception data
- **sensor_type**: Enum (camera, lidar, imu, etc.) - Type of sensor that generated the data
- **timestamp**: DateTime - When the data was captured
- **robot_state**: Object - State of the robot when data was captured
- **raw_data_path**: String - Path to the raw sensor data
- **processed_data**: Object - Processed perception results
  - **detected_objects**: Array - List of detected objects
  - **environment_map**: Object - Map of the environment
  - **robot_pose**: Object - Pose of the robot in the environment