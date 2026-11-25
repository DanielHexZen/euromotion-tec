Open-loop data extracted from ROS 2 bag: openloop_test01
========================================================
Generated 2026-11-05T04:20:28 by ros2bag_to_mat.py v1.0
Recording start: 2026-11-05T01:05:06.943285+00:00   duration: 105.21 s   messages: 214103   namespace: '/mcqueen56'

Conventions
  * Each .mat file holds one MATLAB struct per ROS topic, named after the topic without
    the robot namespace (e.g. /mcqueen56/sensors/core -> sensors_core).
  * Struct fields mirror the ROS message (s.drive.speed, s.pose.pose.position.x, ...).
    Every numeric field is an N x 1 column (N = number of messages) or N x K for arrays.
  * t        = time the message was recorded, seconds since bag start.
    t_header = header.stamp of the message, seconds since bag start (if the message has a header).
  * Every quaternion struct also gets roll/pitch/yaw [rad] fields.
  * Constant strings (e.g. frame_id) are stored once as char, varying ones as N x 1 cell.
  * <bag>_synced.mat resamples the main signals on one uniform time vector for system ID.
    Commands use zero-order hold; sensors use linear interpolation (NaN across data gaps).

Load in MATLAB:
  S   = load('openloop_test01_synced.mat');      % S.t, S.cmd_speed, S.vesc_erpm, ...
  cmd = load('openloop_test01_commands.mat');    % cmd.ackermann_cmd.drive.speed, ...

IMPORTANT unit notes
  * IMU topics (sensors_imu_raw, sensors_imu) are published by the VESC in g and deg/s,
    even sensors_imu_raw which is a sensor_msgs/Imu. The synced file converts to m/s^2 and rad/s.
  * vesc state.speed is electrical RPM. speed[m/s] = (ERPM - offset) / speed_to_erpm_gain
    (real vehicle values at the bottom of this file and in info.gains).

------------------------------------------------------------------------------------------
openloop_test01_commands.mat   [commands] Control inputs sent to the car (Ackermann commands, motor ERPM and servo commands).
------------------------------------------------------------------------------------------
  teleop   <- /mcqueen56/teleop  (ackermann_msgs/msg/AckermannDriveStamped), 25854 msgs, ~246.0 Hz
    Ackermann command from the joystick teleop node.
    teleop.topic [char] = '/mcqueen56/teleop'
    teleop.type [char] = 'ackermann_msgs/msg/AckermannDriveStamped'
    teleop.count [scalar]
    teleop.t [25854x1 double]   -- receive time since bag start [s]
    teleop.t_header [25854x1 double]   -- header.stamp since bag start [s]
    teleop.header
    teleop.header.stamp [25854x1 double]
    teleop.header.frame_id [char] = ''
    teleop.drive
    teleop.drive.steering_angle [25854x1 double]   -- commanded steering angle [rad], + = left
    teleop.drive.steering_angle_velocity [25854x1 double]
    teleop.drive.speed [25854x1 double]   -- commanded speed [m/s]
    teleop.drive.acceleration [25854x1 double]
    teleop.drive.jerk [25854x1 double]
    teleop.recv_time_ns [25854x1 int64]   -- absolute time [ns since Unix epoch]
    teleop.header_time_ns [25854x1 int64]   -- absolute time [ns since Unix epoch]

  sensors_servo_position_command   <- /mcqueen56/sensors/servo_position_command  (std_msgs/msg/Float64), 25227 msgs, ~239.9 Hz
    Servo command echoed back by the VESC driver [0..1].
    sensors_servo_position_command.topic [char] = '/mcqueen56/sensors/servo_position_command'
    sensors_servo_position_command.type [char] = 'std_msgs/msg/Float64'
    sensors_servo_position_command.count [scalar]
    sensors_servo_position_command.t [25227x1 double]   -- receive time since bag start [s]
    sensors_servo_position_command.data [25227x1 double]   -- value (see topic description)
    sensors_servo_position_command.recv_time_ns [25227x1 int64]   -- absolute time [ns since Unix epoch]

  commands_servo_position   <- /mcqueen56/commands/servo/position  (std_msgs/msg/Float64), 25238 msgs, ~239.9 Hz
    Servo command sent to VESC [0..1] (= steering_to_servo_gain * angle + offset).
    commands_servo_position.topic [char] = '/mcqueen56/commands/servo/position'
    commands_servo_position.type [char] = 'std_msgs/msg/Float64'
    commands_servo_position.count [scalar]
    commands_servo_position.t [25238x1 double]   -- receive time since bag start [s]
    commands_servo_position.data [25238x1 double]   -- value (see topic description)
    commands_servo_position.recv_time_ns [25238x1 int64]   -- absolute time [ns since Unix epoch]

  commands_motor_speed   <- /mcqueen56/commands/motor/speed  (std_msgs/msg/Float64), 25238 msgs, ~239.9 Hz
    Motor speed command sent to VESC [ERPM] (= speed_to_erpm_gain * speed + offset).
    commands_motor_speed.topic [char] = '/mcqueen56/commands/motor/speed'
    commands_motor_speed.type [char] = 'std_msgs/msg/Float64'
    commands_motor_speed.count [scalar]
    commands_motor_speed.t [25238x1 double]   -- receive time since bag start [s]
    commands_motor_speed.data [25238x1 double]   -- value (see topic description)
    commands_motor_speed.recv_time_ns [25238x1 int64]   -- absolute time [ns since Unix epoch]

  ackermann_cmd   <- /mcqueen56/ackermann_cmd  (ackermann_msgs/msg/AckermannDriveStamped), 25238 msgs, ~239.9 Hz
    Ackermann command actually executed (output of the mux).
    ackermann_cmd.topic [char] = '/mcqueen56/ackermann_cmd'
    ackermann_cmd.type [char] = 'ackermann_msgs/msg/AckermannDriveStamped'
    ackermann_cmd.count [scalar]
    ackermann_cmd.t [25238x1 double]   -- receive time since bag start [s]
    ackermann_cmd.t_header [25238x1 double]   -- header.stamp since bag start [s]
    ackermann_cmd.header
    ackermann_cmd.header.stamp [25238x1 double]
    ackermann_cmd.header.frame_id [char] = ''
    ackermann_cmd.drive
    ackermann_cmd.drive.steering_angle [25238x1 double]   -- commanded steering angle [rad], + = left
    ackermann_cmd.drive.steering_angle_velocity [25238x1 double]
    ackermann_cmd.drive.speed [25238x1 double]   -- commanded speed [m/s]
    ackermann_cmd.drive.acceleration [25238x1 double]
    ackermann_cmd.drive.jerk [25238x1 double]
    ackermann_cmd.recv_time_ns [25238x1 int64]   -- absolute time [ns since Unix epoch]
    ackermann_cmd.header_time_ns [25238x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_vesc.mat   [vesc] VESC motor-controller telemetry (motor speed, currents, duty cycle, voltage, tachometer) and VESC IMU.
------------------------------------------------------------------------------------------
  sensors_imu   <- /mcqueen56/sensors/imu  (vesc_msgs/msg/VescImuStamped), 5258 msgs, ~50.0 Hz
    VESC on-board IMU (custom message).
    sensors_imu.topic [char] = '/mcqueen56/sensors/imu'
    sensors_imu.type [char] = 'vesc_msgs/msg/VescImuStamped'
    sensors_imu.count [scalar]
    sensors_imu.t [5258x1 double]   -- receive time since bag start [s]
    sensors_imu.t_header [5258x1 double]   -- header.stamp since bag start [s]
    sensors_imu.header
    sensors_imu.header.stamp [5258x1 double]
    sensors_imu.header.frame_id [char] = ''
    sensors_imu.imu
    sensors_imu.imu.ypr   -- Euler angles from VESC IMU [deg] (z ~ yaw; sign convention is the VESC's - prefer orientation.yaw)
    sensors_imu.imu.ypr.x [5258x1 double]
    sensors_imu.imu.ypr.y [5258x1 double]
    sensors_imu.imu.ypr.z [5258x1 double]
    sensors_imu.imu.linear_acceleration   -- acceleration [g]
    sensors_imu.imu.linear_acceleration.x [5258x1 double]
    sensors_imu.imu.linear_acceleration.y [5258x1 double]
    sensors_imu.imu.linear_acceleration.z [5258x1 double]
    sensors_imu.imu.angular_velocity   -- angular rate [deg/s]
    sensors_imu.imu.angular_velocity.x [5258x1 double]
    sensors_imu.imu.angular_velocity.y [5258x1 double]
    sensors_imu.imu.angular_velocity.z [5258x1 double]
    sensors_imu.imu.compass
    sensors_imu.imu.compass.x [5258x1 double]
    sensors_imu.imu.compass.y [5258x1 double]
    sensors_imu.imu.compass.z [5258x1 double]
    sensors_imu.imu.orientation
    sensors_imu.imu.orientation.x [5258x1 double]
    sensors_imu.imu.orientation.y [5258x1 double]
    sensors_imu.imu.orientation.z [5258x1 double]
    sensors_imu.imu.orientation.w [5258x1 double]
    sensors_imu.imu.orientation.roll [5258x1 double]   -- roll computed from quaternion [rad] (added by extractor)
    sensors_imu.imu.orientation.pitch [5258x1 double]   -- pitch computed from quaternion [rad] (added by extractor)
    sensors_imu.imu.orientation.yaw [5258x1 double]   -- yaw computed from quaternion [rad] (added by extractor)
    sensors_imu.recv_time_ns [5258x1 int64]   -- absolute time [ns since Unix epoch]
    sensors_imu.header_time_ns [5258x1 int64]   -- absolute time [ns since Unix epoch]

  sensors_core   <- /mcqueen56/sensors/core  (vesc_msgs/msg/VescStateStamped), 5259 msgs, ~50.0 Hz
    VESC state telemetry.
    sensors_core.topic [char] = '/mcqueen56/sensors/core'
    sensors_core.type [char] = 'vesc_msgs/msg/VescStateStamped'
    sensors_core.count [scalar]
    sensors_core.t [5259x1 double]   -- receive time since bag start [s]
    sensors_core.t_header [5259x1 double]   -- header.stamp since bag start [s]
    sensors_core.header
    sensors_core.header.stamp [5259x1 double]
    sensors_core.header.frame_id [char] = ''
    sensors_core.state
    sensors_core.state.temp_fet [5259x1 double]
    sensors_core.state.temp_motor [5259x1 double]
    sensors_core.state.current_motor [5259x1 double]   -- motor current [A]
    sensors_core.state.current_input [5259x1 double]   -- battery/input current [A]
    sensors_core.state.avg_id [5259x1 double]
    sensors_core.state.avg_iq [5259x1 double]
    sensors_core.state.duty_cycle [5259x1 double]   -- duty cycle [-1..1]
    sensors_core.state.speed [5259x1 double]   -- motor electrical speed [ERPM]
    sensors_core.state.voltage_input [5259x1 double]   -- battery voltage [V]
    sensors_core.state.charge_drawn [5259x1 double]
    sensors_core.state.charge_regen [5259x1 double]
    sensors_core.state.energy_drawn [5259x1 double]
    sensors_core.state.energy_regen [5259x1 double]
    sensors_core.state.displacement [5259x1 double]   -- net tachometer [counts]
    sensors_core.state.distance_traveled [5259x1 double]   -- absolute tachometer [counts]
    sensors_core.state.fault_code [5259x1 double]
    sensors_core.state.pid_pos_now [5259x1 double]   -- rotor position [deg]
    sensors_core.state.controller_id [5259x1 double]
    sensors_core.state.ntc_temp_mosfet1 [5259x1 double]
    sensors_core.state.ntc_temp_mosfet2 [5259x1 double]
    sensors_core.state.ntc_temp_mosfet3 [5259x1 double]
    sensors_core.state.avg_vd [5259x1 double]
    sensors_core.state.avg_vq [5259x1 double]
    sensors_core.recv_time_ns [5259x1 int64]   -- absolute time [ns since Unix epoch]
    sensors_core.header_time_ns [5259x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_imu.mat   [imu] Inertial measurement unit topics (sensor_msgs/Imu).
------------------------------------------------------------------------------------------
  sensors_imu_raw   <- /mcqueen56/sensors/imu/raw  (sensor_msgs/msg/Imu), 5258 msgs, ~50.0 Hz
    VESC on-board IMU as sensor_msgs/Imu. Units are g and deg/s (not SI).
    sensors_imu_raw.topic [char] = '/mcqueen56/sensors/imu/raw'
    sensors_imu_raw.type [char] = 'sensor_msgs/msg/Imu'
    sensors_imu_raw.count [scalar]
    sensors_imu_raw.t [5258x1 double]   -- receive time since bag start [s]
    sensors_imu_raw.t_header [5258x1 double]   -- header.stamp since bag start [s]
    sensors_imu_raw.header
    sensors_imu_raw.header.stamp [5258x1 double]
    sensors_imu_raw.header.frame_id [char] = ''
    sensors_imu_raw.orientation
    sensors_imu_raw.orientation.x [5258x1 double]
    sensors_imu_raw.orientation.y [5258x1 double]
    sensors_imu_raw.orientation.z [5258x1 double]
    sensors_imu_raw.orientation.w [5258x1 double]
    sensors_imu_raw.orientation.roll [5258x1 double]   -- roll computed from quaternion [rad] (added by extractor)
    sensors_imu_raw.orientation.pitch [5258x1 double]   -- pitch computed from quaternion [rad] (added by extractor)
    sensors_imu_raw.orientation.yaw [5258x1 double]   -- yaw computed from quaternion [rad] (added by extractor)
    sensors_imu_raw.orientation_covariance [5258x9 double]   -- row-major 3x3 covariance per row
    sensors_imu_raw.angular_velocity   -- angular rate - VESC IMU reports [deg/s], NOT rad/s
    sensors_imu_raw.angular_velocity.x [5258x1 double]
    sensors_imu_raw.angular_velocity.y [5258x1 double]
    sensors_imu_raw.angular_velocity.z [5258x1 double]
    sensors_imu_raw.angular_velocity_covariance [5258x9 double]   -- row-major 3x3 covariance per row
    sensors_imu_raw.linear_acceleration   -- acceleration - VESC IMU reports [g], NOT m/s^2
    sensors_imu_raw.linear_acceleration.x [5258x1 double]
    sensors_imu_raw.linear_acceleration.y [5258x1 double]
    sensors_imu_raw.linear_acceleration.z [5258x1 double]
    sensors_imu_raw.linear_acceleration_covariance [5258x9 double]   -- row-major 3x3 covariance per row
    sensors_imu_raw.recv_time_ns [5258x1 int64]   -- absolute time [ns since Unix epoch]
    sensors_imu_raw.header_time_ns [5258x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_odometry.mat   [odometry] Wheel odometry (from VESC) and particle-filter localization pose.
------------------------------------------------------------------------------------------
  pf_pose_odom   <- /mcqueen56/pf/pose/odom  (nav_msgs/msg/Odometry), 3073 msgs, ~29.2 Hz
    Particle filter localization (pose in map frame).
    pf_pose_odom.topic [char] = '/mcqueen56/pf/pose/odom'
    pf_pose_odom.type [char] = 'nav_msgs/msg/Odometry'
    pf_pose_odom.count [scalar]
    pf_pose_odom.t [3073x1 double]   -- receive time since bag start [s]
    pf_pose_odom.t_header [3073x1 double]   -- header.stamp since bag start [s]
    pf_pose_odom.header
    pf_pose_odom.header.stamp [3073x1 double]
    pf_pose_odom.header.frame_id [char] = 'map'
    pf_pose_odom.child_frame_id [char] = ''
    pf_pose_odom.pose
    pf_pose_odom.pose.pose
    pf_pose_odom.pose.pose.position   -- position [m] in header.frame_id
    pf_pose_odom.pose.pose.position.x [3073x1 double]
    pf_pose_odom.pose.pose.position.y [3073x1 double]
    pf_pose_odom.pose.pose.position.z [3073x1 double]
    pf_pose_odom.pose.pose.orientation
    pf_pose_odom.pose.pose.orientation.x [3073x1 double]
    pf_pose_odom.pose.pose.orientation.y [3073x1 double]
    pf_pose_odom.pose.pose.orientation.z [3073x1 double]
    pf_pose_odom.pose.pose.orientation.w [3073x1 double]
    pf_pose_odom.pose.pose.orientation.roll [3073x1 double]   -- roll computed from quaternion [rad] (added by extractor)
    pf_pose_odom.pose.pose.orientation.pitch [3073x1 double]   -- pitch computed from quaternion [rad] (added by extractor)
    pf_pose_odom.pose.pose.orientation.yaw [3073x1 double]   -- yaw computed from quaternion [rad] (added by extractor)
    pf_pose_odom.pose.covariance [3073x36 double]   -- row-major 6x6 covariance per row: reshape(row,6,6)'
    pf_pose_odom.twist
    pf_pose_odom.twist.twist
    pf_pose_odom.twist.twist.linear   -- velocity [m/s] in child_frame_id
    pf_pose_odom.twist.twist.linear.x [3073x1 double]
    pf_pose_odom.twist.twist.linear.y [3073x1 double]
    pf_pose_odom.twist.twist.linear.z [3073x1 double]
    pf_pose_odom.twist.twist.angular   -- angular velocity [rad/s]
    pf_pose_odom.twist.twist.angular.x [3073x1 double]
    pf_pose_odom.twist.twist.angular.y [3073x1 double]
    pf_pose_odom.twist.twist.angular.z [3073x1 double]
    pf_pose_odom.twist.covariance [3073x36 double]   -- row-major 6x6 covariance per row: reshape(row,6,6)'
    pf_pose_odom.recv_time_ns [3073x1 int64]   -- absolute time [ns since Unix epoch]
    pf_pose_odom.header_time_ns [3073x1 int64]   -- absolute time [ns since Unix epoch]

  odom   <- /mcqueen56/odom  (nav_msgs/msg/Odometry), 5260 msgs, ~50.0 Hz
    Wheel odometry computed from VESC ERPM and servo command.
    odom.topic [char] = '/mcqueen56/odom'
    odom.type [char] = 'nav_msgs/msg/Odometry'
    odom.count [scalar]
    odom.t [5260x1 double]   -- receive time since bag start [s]
    odom.t_header [5260x1 double]   -- header.stamp since bag start [s]
    odom.header
    odom.header.stamp [5260x1 double]
    odom.header.frame_id [char] = 'mcqueen56/odom'
    odom.child_frame_id [char] = 'mcqueen56/base_link'
    odom.pose
    odom.pose.pose
    odom.pose.pose.position   -- position [m] in header.frame_id
    odom.pose.pose.position.x [5260x1 double]
    odom.pose.pose.position.y [5260x1 double]
    odom.pose.pose.position.z [5260x1 double]
    odom.pose.pose.orientation
    odom.pose.pose.orientation.x [5260x1 double]
    odom.pose.pose.orientation.y [5260x1 double]
    odom.pose.pose.orientation.z [5260x1 double]
    odom.pose.pose.orientation.w [5260x1 double]
    odom.pose.pose.orientation.roll [5260x1 double]   -- roll computed from quaternion [rad] (added by extractor)
    odom.pose.pose.orientation.pitch [5260x1 double]   -- pitch computed from quaternion [rad] (added by extractor)
    odom.pose.pose.orientation.yaw [5260x1 double]   -- yaw computed from quaternion [rad] (added by extractor)
    odom.pose.covariance [5260x36 double]   -- row-major 6x6 covariance per row: reshape(row,6,6)'
    odom.twist
    odom.twist.twist
    odom.twist.twist.linear   -- velocity [m/s] in child_frame_id
    odom.twist.twist.linear.x [5260x1 double]
    odom.twist.twist.linear.y [5260x1 double]
    odom.twist.twist.linear.z [5260x1 double]
    odom.twist.twist.angular   -- angular velocity [rad/s]
    odom.twist.twist.angular.x [5260x1 double]
    odom.twist.twist.angular.y [5260x1 double]
    odom.twist.twist.angular.z [5260x1 double]
    odom.twist.covariance [5260x36 double]   -- row-major 6x6 covariance per row: reshape(row,6,6)'
    odom.recv_time_ns [5260x1 int64]   -- absolute time [ns since Unix epoch]
    odom.header_time_ns [5260x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_joystick.mat   [joystick] Raw joystick input (sensor_msgs/Joy).
------------------------------------------------------------------------------------------
  joy   <- /mcqueen56/joy  (sensor_msgs/msg/Joy), 63197 msgs, ~601.0 Hz
    Raw joystick axes and buttons.
    joy.topic [char] = '/mcqueen56/joy'
    joy.type [char] = 'sensor_msgs/msg/Joy'
    joy.count [scalar]
    joy.t [63197x1 double]   -- receive time since bag start [s]
    joy.t_header [63197x1 double]   -- header.stamp since bag start [s]
    joy.header
    joy.header.stamp [63197x1 double]
    joy.header.frame_id [char] = 'joy'
    joy.axes [63197x8 double]
    joy.buttons [63197x14 double]
    joy.recv_time_ns [63197x1 int64]   -- absolute time [ns since Unix epoch]
    joy.header_time_ns [63197x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_tf.mat   [tf] Coordinate transforms.
------------------------------------------------------------------------------------------
  tf_static   <- /tf_static  (tf2_msgs/msg/TFMessage), 3 msgs, ~216.3 Hz
    Static transforms (all transforms of all messages concatenated, see msg_index).
    tf_static.topic [char] = '/tf_static'
    tf_static.type [char] = 'tf2_msgs/msg/TFMessage'
    tf_static.count [scalar]
    tf_static.t [3x1 double]   -- receive time since bag start [s]
    tf_static.transforms
    tf_static.transforms.t_header [3x1 double]   -- header.stamp since bag start [s]
    tf_static.transforms.msg_index [3x1 double]   -- index of the parent message this row belongs to
    tf_static.transforms.header
    tf_static.transforms.header.stamp [3x1 double]
    tf_static.transforms.header.frame_id [3x1 cell]
    tf_static.transforms.child_frame_id [3x1 cell]
    tf_static.transforms.transform
    tf_static.transforms.transform.translation
    tf_static.transforms.transform.translation.x [3x1 double]
    tf_static.transforms.transform.translation.y [3x1 double]
    tf_static.transforms.transform.translation.z [3x1 double]
    tf_static.transforms.transform.rotation
    tf_static.transforms.transform.rotation.x [3x1 double]
    tf_static.transforms.transform.rotation.y [3x1 double]
    tf_static.transforms.transform.rotation.z [3x1 double]
    tf_static.transforms.transform.rotation.w [3x1 double]
    tf_static.transforms.transform.rotation.roll [3x1 double]   -- roll computed from quaternion [rad] (added by extractor)
    tf_static.transforms.transform.rotation.pitch [3x1 double]   -- pitch computed from quaternion [rad] (added by extractor)
    tf_static.transforms.transform.rotation.yaw [3x1 double]   -- yaw computed from quaternion [rad] (added by extractor)
    tf_static.transforms.header_time_ns [3x1 int64]   -- absolute time [ns since Unix epoch]
    tf_static.recv_time_ns [3x1 int64]   -- absolute time [ns since Unix epoch]

------------------------------------------------------------------------------------------
openloop_test01_synced.mat   [time-synchronized signals @ 100 Hz, time source: header]
------------------------------------------------------------------------------------------
  t                    [s]  Time since bag start   <- common time grid
  cmd_speed            [m/s]  Executed Ackermann speed command   <- ackermann_cmd.drive.speed (zoh)
  cmd_steering         [rad]  Executed Ackermann steering command (+ left)   <- ackermann_cmd.drive.steering_angle (zoh)
  teleop_speed         [m/s]  Teleop (joystick) speed request   <- teleop.drive.speed (zoh)
  teleop_steering      [rad]  Teleop (joystick) steering request   <- teleop.drive.steering_angle (zoh)
  cmd_motor_erpm       [ERPM]  Motor speed command sent to VESC   <- commands_motor_speed.data (zoh)
  cmd_servo            [-]  Servo position command sent to VESC [0..1]   <- commands_servo_position.data (zoh)
  cmd_servo_echo       [-]  Servo command echoed by VESC driver [0..1]   <- sensors_servo_position_command.data (zoh)
  vesc_erpm            [ERPM]  Measured motor electrical speed   <- sensors_core.state.speed (lin)
  vesc_duty            [-]  Duty cycle   <- sensors_core.state.duty_cycle (lin)
  vesc_current_motor   [A]  Motor current   <- sensors_core.state.current_motor (lin)
  vesc_current_input   [A]  Battery/input current   <- sensors_core.state.current_input (lin)
  vesc_iq              [A]  Average q-axis current (torque producing)   <- sensors_core.state.avg_iq (lin)
  vesc_voltage         [V]  Battery voltage   <- sensors_core.state.voltage_input (lin)
  vesc_tacho           [counts]  Net tachometer   <- sensors_core.state.displacement (lin)
  odom_x               [m]  Wheel odometry x (odom frame)   <- odom.pose.pose.position.x (lin)
  odom_y               [m]  Wheel odometry y (odom frame)   <- odom.pose.pose.position.y (lin)
  odom_yaw             [rad]  Wheel odometry heading (unwrapped)   <- odom.pose.pose.orientation.yaw (lin_unwrap)
  odom_vx              [m/s]  Wheel odometry longitudinal speed   <- odom.twist.twist.linear.x (lin)
  odom_vy              [m/s]  Wheel odometry lateral speed   <- odom.twist.twist.linear.y (lin)
  odom_wz              [rad/s]  Wheel odometry yaw rate   <- odom.twist.twist.angular.z (lin)
  pf_x                 [m]  Particle filter x (map frame)   <- pf_pose_odom.pose.pose.position.x (lin)
  pf_y                 [m]  Particle filter y (map frame)   <- pf_pose_odom.pose.pose.position.y (lin)
  pf_yaw               [rad]  Particle filter heading (unwrapped)   <- pf_pose_odom.pose.pose.orientation.yaw (lin_unwrap)
  pf_vx                [m/s]  Particle filter reported speed   <- pf_pose_odom.twist.twist.linear.x (lin)
  imu_ax               [m/s^2]  IMU longitudinal acceleration   <- sensors_imu_raw.linear_acceleration.x * 9.80665 (lin)
  imu_ay               [m/s^2]  IMU lateral acceleration   <- sensors_imu_raw.linear_acceleration.y * 9.80665 (lin)
  imu_az               [m/s^2]  IMU vertical acceleration   <- sensors_imu_raw.linear_acceleration.z * 9.80665 (lin)
  imu_wx               [rad/s]  IMU roll rate   <- sensors_imu_raw.angular_velocity.x * 0.0174533 (lin)
  imu_wy               [rad/s]  IMU pitch rate   <- sensors_imu_raw.angular_velocity.y * 0.0174533 (lin)
  imu_wz               [rad/s]  IMU yaw rate   <- sensors_imu_raw.angular_velocity.z * 0.0174533 (lin)
  imu_yaw              [rad]  IMU heading from orientation (unwrapped)   <- sensors_imu_raw.orientation.yaw (lin_unwrap)
  vesc_speed           [m/s]  Wheel speed from measured ERPM using the vehicle's speed_to_erpm gain   <- (vesc_erpm - 0) / 4450
  servo_steering       [rad]  Steering angle actually commanded to the servo (after servo_min/max clipping)   <- (cmd_servo - 0.50554) / -1.35
  units / description / source : structs with the same fields describing each signal
  segments : intervals with non-zero speed command (t_start, t_end, mean_cmd_speed, ...)
  gains    : vehicle VESC parameters used for conversions (copy of info.gains)

------------------------------------------------------------------------------------------
openloop_test01_info.mat
------------------------------------------------------------------------------------------
  info.bag_name, bag_path, start_time_iso, start_time_ns, duration_s, message_count, namespace
  info.topics : struct of columns (name, type, count, rate_hz, category, variable, file)
  info.gains  : vehicle VESC parameters (erpm/servo mapping, servo limits, wheelbase),
                where they came from, and the values recovered from this bag's commands

Vehicle parameters used:
  source: built-in profile 'mcqueen56': McQueen56 workspace src/f1tenth_system-humble-devel/f1tenth_stack/config/vesc_no_steer_limit.yaml (loaded by no_limit_bringup_launch.py, namespace mcqueen56)
  speed_to_erpm_gain: 4450
  speed_to_erpm_offset: 0
  steering_angle_to_servo_gain: -1.35
  steering_angle_to_servo_offset: 0.50554
  servo_min: 0
  servo_max: 1
  speed_min: -23250
  speed_max: 23250
  wheelbase: 0.324
  erpm  = speed_to_erpm_gain * speed[m/s] + speed_to_erpm_offset
  servo = steering_angle_to_servo_gain * steering[rad] + steering_angle_to_servo_offset
Recovered from this bag's commands (cross-check): speed_to_erpm_gain=4450, steering_angle_to_servo_offset=0.50554, steering_angle_to_servo_gain=-1.35, servo_cmd_min=0.0973951, servo_cmd_max=0.944974
Consistent with bag: yes
