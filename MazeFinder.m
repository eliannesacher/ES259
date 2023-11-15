function MazeFinder(filename)
    %When run, this function should devise a route through the given
    % obstacles and command the UR5e to move along that path. 

    %Inputs: 
        % filename: string with the name of a text file describing obstacle positions.
        % The file should be formatted as: 

        % robot-start-x robot-start-y robot-goal-x robot-goal-y
        % obstacle1-x-min obstacle1-y-min obstacle1-x-max obstacle1-y-max
        % obstacle2-x-min obstacle2-y-min obstacle2-x-max obstacle2-y-max
        % ...
        
        % Note that the first 4 obstacles (lines 2,3,4,5), will be the
        % outer boundaries of the table in the lab.

    %Returns:
        %none

    % Function-Wide Variables
    obstacleOffset = 22.5; %32; % Virtual padding around obstacles to account for end-effector width
    gridSize = 3; %10; % Size of side of each grid square in mm, smaller number means higher resolution
    dt = 0.5;
    
    %Plot to Help Visualize Map (Not Necessary)
    plotRectangle(filename)

    %Determine Shortest Path Through Maze
    [WaypointOutput,lengthPath] = aStar(filename);
    disp(['Length of Path: ',num2str(lengthPath),'mm'])

    % Move The Robot Along Returned Path
    % executeTrajectory(WaypointOutput,filename);
    
    % plot path
    for j = 1:length(WaypointOutput)
         plot(WaypointOutput(1, j), WaypointOutput(2, j), 'b*')
    end

    executeTrajectory(WaypointOutput,filename)
    

    %Functions:

    function [startPos, endPos, obstacles] = processFile(filename)
        %Status: COMPLETE

        %Converts the raw strings in "filename" to a start point, end point,
        %and obstacle matrix.
        %Inputs:
            %filename: a string with the name of the file
        %Returns:
            % startPos: a 1x2 vector of the robot's start position (x,y)
            % endPos: a 1x2 vector of the robot's end position (x,y)
            %obstacles: a matrix containing all the obstacles in the file
            %in the format:
            %[ c1 c2 c3 c4;
            %... ]
            %where c1...c4 are the coordinates of each rectangular
            %obstacle, starting in the upper leftmost corner (when facing
            %the robot, sitting at the computer) and going clockwise
            
        % Open file
        file = splitlines(fileread(filename));

        % the delimiter for each value is 1 space.
        numMat = str2double(split(file, ' '));
        startPos = numMat(1, 1:2);
        endPos = numMat(1, 3:4);
        obstacles = numMat(2:end, :);        
    end

    function plotit = plotRectangle(filename)
        %Status: COMPLETE

        %This function helps for visualizations, but is not used for
        %anything else
        [startPos, endPos, Obstacles] = processFile(filename);
        
        % plot the obstacles as seen in the workspace
        figure()
        axis equal
        hold on
        for i = 1:size(Obstacles, 1)
            lowCorner = Obstacles(i, 1:2);
            width = Obstacles(i, 3) - Obstacles(i, 1);
            height = Obstacles(i, 4) - Obstacles(i, 2);
            pos = [lowCorner width height];
            rectangle('Position', pos,'FaceColor',[0 0.5 1])
        end
        
        % plot the start and end positon (center of robot)
        plot(startPos(1, 1), startPos(1, 2), 'r*')
        plot(endPos(1, 1), endPos(1, 2), 'g*')
        legend('start', 'goal')
    
    end

    function inObstacle = isInObstacle(x,y, obstacleList)
        %Status: COMPLETE

        %Checks if a point is inside of an obstacle or not
        %Inputs:
            %x (double): an x coordinate in the frame of the table
            %y (double): a y coordinate in the frame of the table
	    %obstacleList: a matrix in the same format used elsewhere in this file
        %Outputs:
            %inObstacle (boolean): true if the point is in obstacle
        inObstacle = false;
        for i = 1:height(obstacleList)
            minX = obstacleList(i,1);
            maxX = obstacleList(i,3);
            minY = obstacleList(i,2);
            maxY = obstacleList(i,4);
            if (x >= (minX-obstacleOffset) && x <= (maxX+obstacleOffset))
                if (y >= (minY-obstacleOffset) && y <= (maxY+obstacleOffset))
                    inObstacle = true;
                end
            end
        end
    end


    function index = equal_coords(A,pos)
        %Status: COMPLETE

        %Potentially useful helper function for returning first index from a list that matches
        %the desired position

        %Inputs: A (2xn matrix of n points [x1,x2,...,xn;y1,y2,...yn])
        %        pos (1x2 vector [x,y])
        %Returns: index (integer) corresponding to the index (values range 
        % from 1 to n, inclusive) of the first position in the list that has the same
        % value as pos.
        index = [];
        for j=1:size(A,2)
            if abs(pos(1)-A(1,j)) < 1e-6 && abs(pos(2)-A(2,j)) < 1e-6
                index = j;
                return
            end
        end
    end

    function [WaypointOutput,lengthPath] = aStar(filename)
        %Status: COMPLETE
        
        %Runs A* to find a path and return waypoints
        %Useful Resource:
        %https://www.geeksforgeeks.org/a-search-algorithm/
        %Outputs: 
            % wayPoints: a matrix of waypoints [w1 w2 w3 ...] that the end
            % effector should travel between. Each waypoint is given in the
            % form [x; y]

        %TODO (Start early, this will take a while)
        [startPos, endPos, obstacleList] = processFile(filename);
        % OpenList = [x,y,parentx,parenty,f,g,h];
        openList = [startPos(1),startPos(2),startPos(1),startPos(2),0,0,0];
        closedList = [];
        isGotToEndpoint = false;
        while ~isempty(openList)
            if isGotToEndpoint
                break;
            end
            fMin = openList(1,5);
            qInx = 1; 
            for i = 1:height(openList)
                fi = openList(i,5);
                if fi < fMin
                    fMin = fi;
                    qInx = i;
                end
            end
            q = openList(qInx,:);
            openList(qInx,:) = [];
            qSuccessors = getSuccessors(q,endPos);
            for i = 1:height(qSuccessors)
                if isInObstacle(qSuccessors(i,1),qSuccessors(i,2), obstacleList) == false
                    if isInEndPoint([qSuccessors(i,1),qSuccessors(i,2)],endPos) == false
                        inx1 = equal_coords(openList(:,1:2)',qSuccessors(i,1:2));
                        if ~isempty(closedList)
                            inx2 = equal_coords(closedList(:,1:2)',qSuccessors(i,1:2));
                        else
                            inx2 = [];
                        end
                        if ~isempty(inx1) && openList(inx1,5) > qSuccessors(i,5)
                            openList(inx1,:) = qSuccessors(i,:);
                        elseif ~isempty(inx2) && closedList(inx2,5) > qSuccessors(i,5)
                            closedList = [closedList;qSuccessors(i,:)];
                        elseif isempty(inx1) && isempty(inx2)
                            openList = [openList;qSuccessors(i,:)];
                        end
                        
                    else
                        % in this case, since we want to limit run time, we
                        % have the function break when it gets to the
                        % endpoint
                        isGotToEndpoint = true;
                        break
                    end
                end
            end
            closedList = [closedList;q];
        end
        % since the way I programed it insures that the last point added is 
        % the destination point I don't need to check if it arrived at the 
        % destination and I know where to start the parent search from
        inx = height(closedList);
        WaypointOutput = [];
        lengthPath = 0;
        while 1
            WaypointOutput = [closedList(inx,1:2);WaypointOutput];
            lengthPath = lengthPath + compG(closedList(inx,:));
            parCoords = closedList(inx,3:4);
            closedList(inx,:) = [];
            if sqrt((parCoords(1)-startPos(1))^2+(parCoords(2)-startPos(2))^2) <= 1e-3
                break;
            end
            inx = equal_coords(closedList(:,1:2)',parCoords);
        end
        WaypointOutput = [startPos;WaypointOutput];
        WaypointOutput = WaypointOutput';
    end

    function inEndPoint = isInEndPoint(point, endPoint)
        %Status: COMPLETE

        %Checks if a point is close enough to the end-point or not
        %Inputs:
            % point: containing the x and y coordinates of the current point
	        % endPoint: containing the x and y coordinates of the endPose
        %Outputs:
            % inEndPoint (boolean): true if the point is close enough end-point
        inEndPoint = false;
        if sqrt((endPoint(1)-point(1))^2+(endPoint(2)-point(2))^2) <= gridSize
            inEndPoint = true;
        end
    end

    function [s] = getSuccessors(parent,endPoint)
        %Status: COMPLETE
        
        %Finds 8 successors of a given parent point
        %Inputs:
            % parent = the parent point to which we'll find successors
        %Outputs: 
            % s = a list of 8 sucessor points to input point

        %TODO 
        xPar = parent(1);
        yPar = parent(2);
        s = [];
        s = [s;[xPar-gridSize,yPar+gridSize,xPar,yPar,0,0,0]];
        s = [s;[xPar,yPar+gridSize,xPar,yPar,0,0,0]];
        s = [s;[xPar+gridSize,yPar+gridSize,xPar,yPar,0,0,0]];
        s = [s;[xPar-gridSize,yPar,xPar,yPar,0,0,0]];
        s = [s;[xPar+gridSize,yPar,xPar,yPar,0,0,0]];
        s = [s;[xPar-gridSize,yPar-gridSize,xPar,yPar,0,0,0]];
        s = [s;[xPar,yPar-gridSize,xPar,yPar,0,0,0]];
        s = [s;[xPar+gridSize,yPar-gridSize,xPar,yPar,0,0,0]];

        for i = 1:height(s)
            s(i,6) = compG(s(i,:))+parent(6);
            s(i,7) = compH(s(i,:),endPoint);
            s(i,5) = s(i,6) + s(i,7);
        end
    end

    function g = compG(point)
        %Status: COMPLETE
        
        %Finds the g value of a point
        %Inputs:
            % point = containing the x and y coordinates of the current point
        %Outputs: 
            % g = the movement cost to move from the starting point to a given
            % square on the grid, following the path generated to get there. 

        %TODO 
        g = sqrt((point(1)-point(3))^2+(point(2)-point(4))^2);
    end

    function h = compH(point,endPoint)
        %Status: COMPLETE
        
        %Finds the h value of a point
        %Inputs:
            % point = containing the x and y coordinates of the current point
            % endPoint: containing the x and y coordinates of the endPose
        %Outputs: 
            % h = the estimated movement cost to move from that given square
            % on the grid to the final destination. 

        %TODO 
        h = sqrt((point(1)-endPoint(1))^2+(point(2)-endPoint(2))^2);
    end

    function executeTrajectory(wayPoints,filename)
        %Status: COMPLETE
        
        %Moves the robot to the initialization position.
        %Translates the robot to the start position [x,y] while remaining
        %in the raised z plane.
        %Lowers the robot straight down to the specified z plane.
        %Moves the robot through the list of provided waypoints in the
        %specified z plane.
        %Raises the robot straight up to the raised z plane.
        %Moves the robot to the initialization position.

        %TODO
        armCmd = rospublisher('scaled_pos_joint_traj_controller/command');
        shapeMsg = rosmessage(armCmd);
        shapeMsg.JointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

        % Use inverse kinematics function to determine corresponding joint angles:
        N = length(wayPoints);
        theta = zeros(6,N);
        
        thetainit = zeros(6,2);
        
        % For the first waypoint, we provide a guess for the corresponding pose; for the rest, the previous pose is a good guess for the next one
        thetainit(:,1) = IK([[-1,0,0,665];[0,1,0,0];[0,0,-1,100];[0,0,0,1]],[-pi/18,-pi/3,pi/2,-2*pi/3,-pi/2,4*pi/9]);        
        
        % maintaining that z-position, move it to the x and y coordinates of the start point;
        thetainit(:,2) = IK([[-1,0,0,wayPoints(1,1)];[0,1,0,wayPoints(2,1)];[0,0,-1,100];[0,0,0,1]],thetainit(:,1));
        
        % lower it straight down to z = -80mm.
        theta(:,1) = IK([[-1,0,0,wayPoints(1,1)];[0,1,0,wayPoints(2,1)];[0,0,-1,-80];[0,0,0,1]],thetainit(:,2));        
        
        for i=2:N
            theta(:,i) = IK([[-1,0,0,wayPoints(1,i)];[0,1,0,wayPoints(2,i)];[0,0,-1,-80];[0,0,0,1]],theta(:,i-1));
        end

        % Now there are the two ways to proceed described in the lab handout. To use the usual position controller:
        for i=1:2
            commandlist1(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            commandlist1(i).Positions = thetainit(:,i);
            commandlist1(i).TimeFromStart.Sec = floor((i-1)/round(1/dt));
            commandlist1(i).TimeFromStart.Nsec = mod(i-1,round(1/dt))*dt*1e9;
            % Note that both Sec and Nsec need to be integers, and Nsec must be less than 1e9
            if i<2
            % ** Set the desired joint velocities at each waypoint up until the last:
                commandlist1(i).Velocities = ((thetainit(:,i+1)-thetainit(:,i))/dt) ;
            else
            % (but at the last waypoint, the joint velocity should be 0)
               commandlist1(i).Velocities = zeros(6,1);
            end
        end
        shapeMsg.Points = commandlist1;
        send(armCmd,shapeMsg);
        
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
    end

end
