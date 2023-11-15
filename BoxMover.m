% (The below assumes that the environment is already set up with the armCmd publisher to the arm controller)
% Set this variable to 0 while working on the program outside the lab, and change it to 1 when on the real robot:
InLab = 1;

% (x,y) locations in robot coordinates of the top left and bottom right of the corresponding marker blocks:
topcorner = [340,-400];
bottomcorner = [715,425];

% Set up a publisher to send the gripper commands:
gripperpub = rospublisher('/gripper_service_es159','std_msgs/Float32MultiArray');
gripperMsg = rosmessage('std_msgs/Float32MultiArray');

% Create a message that corresponds to moving the arm out of the way of the camera field of view:
if InLab
    armCmd = rospublisher('scaled_pos_joint_traj_controller/command');
else
    armCmd = rospublisher('/arm_controller/command');
end
OriginalMsg = rosmessage(armCmd);
OriginalMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};
originalpoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
originalpoint.Positions = [0 -pi/2 0 0 0 0];
originalpoint.Velocities = [0 0 0 0 0 0];
originalpoint.TimeFromStart.Sec = 5;
OriginalMsg.Points = [originalpoint];

% A position subscriber, like we used in lab 1, can be helpful in letting us make sure a movement command has finished before doing the next thing:
if InLab
  possub = rossubscriber('tf');
else
  possub = rossubscriber('/gazebo/link_states');
end

% Tell the arm to move out of the way of the camera, and wait until it's finished moving to take a snapshot:
send(armCmd, OriginalMsg);

% If in the physical lab, set up a subscriber to the physical camera, and take a snapshot; otherwise, load the sample snapshot:
if InLab
  CamSub = rossubscriber('/usb_cam/image_raw');
  CamMsg = receive(CamSub);
  
  % get pixel data
  data = CamMsg.Data;
else
  loadedCamMsg = load('Lab5snapshot.mat');
  CamMsg = loadedCamMsg.CamMsg;
  
  % get pixel data
  data = CamMsg.Pixels;
end
% First separate out the three color channels from the .Data field of CamMsg:
RedVals = data(1:3:end);
GreenVals = data(2:3:end);
BlueVals = data(3:3:end);

Red = reshape(RedVals,[CamMsg.Width,CamMsg.Height])';
Green = reshape(GreenVals,[CamMsg.Width,CamMsg.Height])';
Blue = reshape(BlueVals,[CamMsg.Width,CamMsg.Height])';

% Now do the image processing described in the Lab 5 handout:

% Crop the images:
Red = Red(151:400,51:550);
Green = Green(151:400,51:550);
Blue = Blue(151:400,51:550);

% Threshold the images to get binary-valued matrices indicating where in the image each color channel is bright:
RedThresh = 200;
GreenThresh = 150;
BlueThresh = 170;
DarkThresh = 125;

% Make binary-valued matrices that show where in the image each color appears (by itself, not as a component of a composite color):
RedOnly = zeros(height(Red),width(Red));
GreenOnly = zeros(height(Red),width(Red));
BlueOnly = zeros(height(Red),width(Red));
for i = 1:height(Red)
    for j = 1:width(Red)
        if Red(i,j) > RedThresh && Green(i,j) <= DarkThresh && Blue(i,j) <= DarkThresh
            RedOnly(i,j) = 1;
        elseif Green(i,j) > GreenThresh && Red(i,j) <= DarkThresh && Blue(i,j) <= DarkThresh
            GreenOnly(i,j) = 1;
        elseif Blue(i,j) > BlueThresh && Green(i,j) <= DarkThresh && Red(i,j) <= DarkThresh
            BlueOnly(i,j) = 1;
        end
    end
end

% Find the centers of the (blue) block and (green) platform:
BlockPosInPixels = findCenter(BlueOnly);
PlatformPosInPixels = findCenter(GreenOnly);

% Find the top left cornern of the top right red block and
% the buttom left corner of the buttom left red block
RedOnes = find(RedOnly>0);
topRightRed = getVecPose(RedOnes(1),RedOnly);
bottomLeftRed = getVecPose(RedOnes(length(RedOnes)),RedOnly);

% Convert from pixels to robot coordinates, using the 'topcorner' and 'bottomcorner' variables:
BlockPos = pixToRobCoord(topcorner,bottomcorner,topRightRed,bottomLeftRed,BlockPosInPixels);
PlatformPos = pixToRobCoord(topcorner,bottomcorner,topRightRed,bottomLeftRed,PlatformPosInPixels);

% Now we're up to section 4 of the pre-lab in the lab handout: Commanding the Robot

% Some useful values:
hoverz = 100; % Height at which arm will not hit objects
targetblockz = -130; % Height at which arm will grasp objects
platformz = -100; % Height at which arm and carried object will clear platform
closedgripper = 255; % Value to send the gripper to close fully
opengripper = 0; % Value to send the gripper to open fully
blockgripper = 147; % Value to send the gripper to close on the block
gripperspeed = 55; % Use this value for speed
gripperforce = 55; % Use this value for force

% Move the end-effector to the initialization position given:
Tinitial = [-1 0 0 665; 0 1 0 0; 0 0 -1 100; 0 0 0 1]; % from Lab 4
Rinitial = Tinitial(1:3,1:3);
guessinitial = [-10 -60 90 -120 -90 80]*(pi/180);

% Use inverse kinematics function to come up with a set of joint angles that will move the end-effector to the desired position, and then send a corresponding message to the arm**
thetaInitial = IK(Tinitial,guessinitial);
TrajPose = rosmessage('trajectory_msgs/JointTrajectoryPoint');
TrajPose.Positions = thetaInitial(:,1);
TrajPose.Velocities = [0,0,0,0,0,0];
TrajPose.TimeFromStart.Sec = 2;
TrajMsg = rosmessage(armCmd);
TrajMsg.JointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Keep that z-value, and move x and y to BlockPos:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a trajectory in the horizontal plane**
TBlockPos = [-1 0 0 BlockPos(2); 0 1 0 BlockPos(1); 0 0 -1 100; 0 0 0 1];
thetaBlockPos = IK(TBlockPos,thetaInitial(:,1));
TrajPose.Positions = thetaBlockPos;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Open the gripper:
gripperMsg.Data = [opengripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5); % wait for gripper to finish moving

% Lower straight down to z=targetblockz:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a vertical trajectory**
TBlockPosDown = [-1 0 0 BlockPos(2); 0 1 0 BlockPos(1); 0 0 -1 targetblockz; 0 0 0 1];
thetaBlockPosDown = IK(TBlockPosDown,thetaBlockPos(:,1));
TrajPose.Positions = thetaBlockPosDown;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Close the gripper on the block:
gripperMsg.Data = [blockgripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5); % wait for gripper to finish moving

% Raise straight up to z=platformz:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a vertical trajectory**
TBlockPosUp = [-1 0 0 BlockPos(2); 0 1 0 BlockPos(1); 0 0 -1 platformz; 0 0 0 1];
thetaBlockPosUp = IK(TBlockPosUp,thetaBlockPosDown(:,1));
TrajPose.Positions = thetaBlockPosUp;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Keep that z-value, and move x and y to PlatformPos:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a trajectory in the horizontal plane**
TPlatformPos = [-1 0 0 PlatformPos(2); 0 1 0 PlatformPos(1); 0 0 -1 platformz; 0 0 0 1];
thetaPlatformPos = IK(TPlatformPos,thetaBlockPosUp(:,1));
TrajPose.Positions = thetaPlatformPos;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Open the gripper:
gripperMsg.Data = [opengripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving

% Raise straight up to z=hoverz:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a vertical trajectory**
TPlatformPosHover = [-1 0 0 PlatformPos(2); 0 1 0 PlatformPos(1); 0 0 -1 hoverz; 0 0 0 1];
thetaPlatformPosHover = IK(TPlatformPosHover,thetaPlatformPos(:,1));
TrajPose.Positions = thetaPlatformPosHover;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Keep that z-value, and move x and y to their values from the initialization position:
% **Code is missing here ... use path-planning code from Lab 4 to move the gripper along a trajectory in the horizontal plane**
thetaBackToStart = IK(Tinitial,thetaPlatformPosHover(:,1));
TrajPose.Positions = thetaBackToStart;
TrajMsg.Points = [TrajPose];
send(armCmd,TrajMsg);
pause(5);

% Close the gripper:
gripperMsg.Data = [closedgripper gripperspeed gripperforce];
send(gripperpub,gripperMsg);
pause(5) % wait for gripper to finish moving

% Return the robot arm to the original position:
send(armCmd, OriginalMsg);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function centerVec = findCenter(A)
% finds the center of a chunck of ones within a larger matrix
f = find(A > 0);
rows = [];
cols = [];
for i = 1:length(f)
    vec = getVecPose(f(i),A);
    if(~ismember(vec(1),rows))
        rows = [rows,vec(1)];
    end
    if(~ismember(vec(2),cols))
        cols = [cols,vec(2)];
    end
end
centerVec = [sum(rows,'all')/length(rows), sum(cols,'all')/length(cols)];
end

function vecPose = getVecPose(pose,A)
% gets the position in terms of rows and columbs given a flaten column index
row = mod(pose,height(A));
if (row == 0)
    row = height(A);
end
col = ceil(pose/height(A));
vecPose = [row,col];
end

function robCoord = pixToRobCoord(topcorner,bottomcorner,topRightRed,bottomLeftRed,pixVec)
% converts pixel coordinates to robot coordinates using the edges pixel
% difference and robot coordinate difference
mmPerCol = (abs(bottomcorner(1) - topcorner(1))) / (abs(bottomLeftRed(1) - topRightRed(1)));
mmPerRow = (abs(bottomcorner(2) - topcorner(2))) / (abs(bottomLeftRed(2) - topRightRed(2)));
col = ((pixVec(1) - topRightRed(1)) * mmPerRow) + topcorner(1);
row = ((pixVec(2) - topRightRed(2)) * mmPerCol) + topcorner(2);
robCoord = [row,col];
end
