% Inverse velocity kinematics to trace out some trajectory.

% time between coordinates
dt = 1;

% Move the robot arm
%rosinit;
armCmd = rospublisher('scaled_pos_joint_traj_controller/command');
shapeMsg = rosmessage(armCmd);
shapeMsg.JointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Number of waypoints in trajectory:
N = 16;

% Adjust to make reachable
adjust = 30.0;

% Set a series of waypoints with desired poses and associated times:
waypoints = [linspace(1,adjust*N/4,N/4),adjust*(N/4)*ones(1,N/4),linspace(adjust*N/4,1,N/4),ones(1,N/4);
             400*ones(1,N/4),linspace(400,400+adjust*N/4,N/4),(400+adjust*N/4)*ones(1,N/4),linspace(400+adjust*N/4,400,N/4)];

% Use inverse kinematics function to determine corresponding joint angles:
theta = zeros(6,N);
% For the first waypoint, we provide a guess for the corresponding pose; for the rest, the previous pose is a good guess for the next one
theta(:,1) = IK([1,0,0,900;0,1,0,waypoints(1,1);0,0,1,waypoints(2,1);0,0,0,1],[0,0,-pi/3,2*pi/3,-pi/2,0]);

for i=2:N
  theta(:,i) = IK([1,0,0,900;0,1,0,waypoints(1,i);0,0,1,waypoints(2,i);0,0,0,1],theta(:,i-1));
end

% Alternatively, to use the pure velocity controller:

% Make sure velCmd, velMsg, and jSub have been created, as described in the handout:
 velCmd = rospublisher('/joint_group_vel_controller/command');
 velMsg = rosmessage(velCmd);
 jSub = rossubscriber('joint_states');

for i=1:size(theta,2)-1
  % get current position:
  jMsg = receive(jSub);
  % switch the order of the joints in the message that returns, to match the actual joint order:
  angles = jMsg.Position([3 2 1 4 5 6]);
  % set the velocity based on the next desired position and the actual current position:
  velMsg.Data = theta(:,i + 1) - angles;
  % execute the velocity command:
  send(velCmd,velMsg)
  % wait until the next time step:
  pause(dt)
end

% stop the robot!
velMsg.Data = zeros(6,1);
send(velCmd,velMsg)
