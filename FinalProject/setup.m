% initialize
%rosinit;

% set up the initiat joint angles
% initJoinAng = [pi -0.75*pi 0 -1.25*pi -0.5*pi -0.985*pi]; % short setup
initJoinAng = [pi -pi 0 -pi -0.5*pi -0.985*pi]; % long setup


% Set up a publisher to send the gripper commands:
gripperpub = rospublisher('/gripper_service_es159','std_msgs/Float32MultiArray');
gripperMsg = rosmessage('std_msgs/Float32MultiArray');

% Create a message that corresponds to moving to the home position
armCmd = rospublisher('scaled_pos_joint_traj_controller/command');

OriginalMsg = rosmessage(armCmd);
OriginalMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
originalpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
originalpoint.Positions = initJoinAng;
originalpoint.Velocities = [0 0 0 0 0 0];
originalpoint.TimeFromStart.Sec = 5;
OriginalMsg.Points = [originalpoint];

% A position subscriber, like we used in lab 1, can be helpful in letting us make sure a movement command has finished before doing the next thing:
possub = rossubscriber('tf');

% Tell the arm to move out of the way of the camera, and wait until it's finished moving to take a snapshot:
send(armCmd, OriginalMsg);
pause(5); % wait until the arm finishes moving

% init gripper control variables
closegripper = 0.4*250;
gripperspeed = 155;
gripperforce = 155;

% Close the gripper:
gripperMsg.Data = [closegripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving 