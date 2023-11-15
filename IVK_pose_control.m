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

% Now there are the two ways to proceed described in the lab handout. To use the usual position controller:
for i=1:N
 commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 commandlist(i).Positions = theta(:,i);
 commandlist(i).TimeFromStart.Sec = floor((i-1)/round(1/dt));
 commandlist(i).TimeFromStart.Nsec = mod(i-1,round(1/dt))*dt*1e9;
 % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
 if i<N
% ** Set the desired joint velocities at each waypoint up until the last:
   commandlist(i).Velocities = ((theta(:,i+1)-theta(:,i))/dt) ;
 else
% (but at the last waypoint, the joint velocity should be 0)
   commandlist(i).Velocities = zeros(6,1);
 end
end
shapeMsg.Points = commandlist;
send(armCmd,shapeMsg);
