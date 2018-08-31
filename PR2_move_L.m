% lArm is the right arm object to control, rGoalMsg is the msg to send
% the client
[lArm, lGoalMsg] = rosactionclient('l_arm_controller/joint_trajectory_action');
% wait to the connection
waitForServer(lArm);

% link joint names
% to get the joint's names use rosparam get /r_arm_controller/joints
lGoalMsg.Trajectory.JointNames = {'l_shoulder_pan_joint', ...
                                   'l_shoulder_lift_joint', ...
                                   'l_upper_arm_roll_joint', ...
                                   'l_elbow_flex_joint',...
                                   'l_forearm_roll_joint',...
                                   'l_wrist_flex_joint',...
                                   'l_wrist_roll_joint'};

% Create setpoints for movementation
% Point 1 - start point
tjPoint1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = zeros(1,7);
tjPoint1.Velocities = zeros(1,7);
tjPoint1.TimeFromStart = rosduration(1.0);

% Point 2 - set point
tjPoint3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [1.0 0.2 -0.1 -1.2 1.5 -0.3 0.5];
tjPoint3.Velocities = zeros(1,7);
tjPoint3.TimeFromStart = rosduration(2.0);

%load the msg with the points (start, goal)
lGoalMsg.Trajectory.Points = [tjPoint1,tjPoint3];

%%
%send the command
%sendGoalAndWait waits for the action to end
sendGoal(lArm,lGoalMsg);
