function [GoalPosePos,BallPos] = imageProcess()
% This function processes the image given by the in-lab camera and returns
% the positions of the forced equilibrium point and the position of the
% ball. Note: it returns these values in pixels rather than positions,
% since the controller is adjusted in a way that it doesn't depend on the
% units of these positions.

% Set up a subscriber to the physical camera, and take a snapshot; otherwise, load the sample snapshot:
CamSub = rossubscriber('/usb_cam/image_raw');
CamMsg = receive(CamSub);

% get pixel data
data = CamMsg.Data;

% First separate out the three color channels from the .Data field of CamMsg:
RedVals = data(1:3:end);
BlueVals = data(3:3:end);

Red = reshape(RedVals,[CamMsg.Width,CamMsg.Height])';
Blue = reshape(BlueVals,[CamMsg.Width,CamMsg.Height])';

% Crop image to show only the platform
% min = 160; % Short set-up
% max = 280; % Short set-up
min = 340; % long set-up
max = 400; % long set-up
Red = Red(min:max,:); 
Blue = Blue(min:max,:); 

% Threshold the images to get binary-valued matrices indicating where in the image each color channel is bright:
RedThresh = 100;
BlueThresh = 80;
DarkThresh = 70;

% Make binary-valued matrices that show where in the image each color appears (by itself, not as a component of a composite color):
RedOnly = zeros(height(Red),width(Red));
BlueOnly = zeros(height(Red),width(Red));
for i = 1:height(Red)
    for j = 1:width(Red)
        if Red(i,j) > RedThresh && Blue(i,j) <= DarkThresh
            RedOnly(i,j) = 1;
        elseif Blue(i,j) > BlueThresh && Red(i,j) <= DarkThresh
            BlueOnly(i,j) = 1;
        end
    end
end

% % For testing the thresh holds uncomment the flowing 5 lines of code.
% figure;
% image(100*BlueOnly);
% 
% figure;
% image(100*RedOnly);


% Find the top left cornern of the top right red corner and
% the buttom left corner of the buttom left red corner
RedOnes = find(RedOnly>0);
topRed = getVecPose(RedOnes(1),RedOnly);
bottomRed = getVecPose(RedOnes(length(RedOnes)),RedOnly);


% Find the centers of the (blue) ball and (middle of reds) goal position (IN PIXELS):
GoalPosePos = [(bottomRed(1)+topRed(1))/2,(bottomRed(2)+topRed(2))/2];
BallPos = findCenter(BlueOnly);


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
end