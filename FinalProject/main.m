% Set constants
dt = 1; % in nano sec
% PID constants
kp = 1;
ki = 0.01;
kd = 0.1;

% Create a message that corresponds to moving to the home position
armCmd = rospublisher('scaled_pos_joint_traj_controller/command');

TrajMsg = rosmessage(armCmd);
TrajMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
TrajPose = rosmessage('trajectory_msgs/JointTrajectoryPoint');
TrajPose.Velocities = [0 0 0 0 0 0];
TrajPose.TimeFromStart.Nsec = dt;

% Get the initial state of the system
[GoalPosePos,BallPos] = imageProcess();

% Produce the lists of distance between the ball and the target (aka the
% error of the state vector)
distErrs = [(GoalPosePos(2)-BallPos(2))/100000];

% initialize the previous angle
jointang = initJoinAng;
% Run until the program quits
while(true)
    % Get the image processing result (aka the state) of the system every dt
    [DontCare,BallPosNew] = imageProcess();
    % Check that there is a value for the ball's position before updating
    if ~isnan(BallPosNew) 
        BallPos = BallPosNew;
    end
    % Get the state error vector
    distErrs = [distErrs,(GoalPosePos(2)-BallPos(2))/100000];
    % Produce the controller angle based on PID control
    theta = kp*distErrs(length(distErrs)) + ki*sum(distErrs,'all') + kd*(distErrs(length(distErrs))-distErrs(length(distErrs)-1));
    % Rotate the wrist angle by theta
    jointang(6) = jointang(6) - theta;
    % limit the angles
    anglim = 0.1;
    if jointang(6) < initJoinAng(6)-anglim
        jointang(6) = initJoinAng(6)-0.5*anglim;
    elseif jointang(6) > initJoinAng(6)+anglim
        jointang(6) = initJoinAng(6)+0.5*anglim;
    end
    % Move the robot arm
    TrajPose.Positions = jointang;
    TrajMsg.Points = [TrajPose];
    send(armCmd,TrajMsg);
    % Wait till it moves
    pause(dt*1e-9);
end