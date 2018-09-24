% rArm is the right arm object to control, rGoalMsg is the msg to send
% the client

[rArm, rGoalMsg] = rosactionclient('r_arm_controller/joint_trajectory_action');
% wait to the connection
waitForServer(rArm);

% link joint names
% to get the joint's names use rosparam get /r_arm_controller/joints
rGoalMsg.Trajectory.JointNames = {'r_shoulder_pan_joint', ...
                                   'r_shoulder_lift_joint', ...
                                   'r_upper_arm_roll_joint', ...
                                   'r_elbow_flex_joint',...
                                   'r_forearm_roll_joint',...
                                   'r_wrist_flex_joint',...
                                   'r_wrist_roll_joint'};

% Create setpoints for movementation
% Point 1 - first point
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = zeros(1,7);
tjPoint1.Velocities = zeros(1,7);
tjPoint1.TimeFromStart = rosduration(1.0);

% Point 2 - second point
tjPoint2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [-1.0 0.2 0.1 -1.2 -1.5 -0.3 -0.5];
tjPoint2.Velocities = zeros(1,7);
tjPoint2.TimeFromStart = rosduration(2.0);

%load the msg with the points (fisrt, second, third, ...)
rGoalMsg.Trajectory.Points = [tjPoint1,tjPoint2];

%send the command and wait for it to end
sendGoalAndWait(rArm,rGoalMsg);
