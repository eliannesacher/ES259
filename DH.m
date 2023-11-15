function pos = DH(th)
% Uses the Denavit-Hartenberg method to determine the final position of
% the end effector using the joint positions of the robot. 
% this is a 6x1 matrix describing the robot's joint positions.

% The given dimensions of the robot:
H1 = 162.5;
H2 = 99.7;
L1 = 425;
L2 = 392;
W1 = 133.3;
W2 = 259.6;

% Create a 6x4 matrix of those values here:
DHmat = [th(1) 0 H1 -pi/2;
        th(2) L1 0 0;
        th(3) L2 0 0;
        th(4) 0 W1 -pi/2;
        th(5) 0 H2, pi/2;
        th(6)+pi 0 W2 0];

% Since in the D-H formulation, the transformation that occurs at each
% joint is a product of four simpler transformations (each a pure
% translation or rotation, with respect to a single coordinate axis),
% create helper functions for performing those simpler transformations:

function Tret = RX(thet)
Tret = [1 0 0 0;
       0 cos(thet) -sin(thet) 0;
       0 sin(thet) cos(thet) 0;
       0 0 0 1];
end

function Tret = RZ(thet)
Tret = [cos(thet) -sin(thet) 0 0;
       sin(thet) cos(thet) 0 0;
       0 0 1 0;
       0 0 0 1];
end

function Tret = TX(dist)
Tret = [1 0 0 dist;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
end

function Tret = TZ(dist)
Tret = [1 0 0 0;
       0 1 0 0;
       0 0 1 dist;
       0 0 0 1];
end

% Now use those functions and the D-H table to calculate the
% transformation matrix for each successive joint relative to the last:
T1 = RZ(DHmat(1,1))*TZ(DHmat(1,3))*TX(DHmat(1,2))*RX(DHmat(1,4));
T2 = RZ(DHmat(2,1))*TZ(DHmat(2,3))*TX(DHmat(2,2))*RX(DHmat(2,4));
T3 = RZ(DHmat(3,1))*TZ(DHmat(3,3))*TX(DHmat(3,2))*RX(DHmat(3,4));
T4 = RZ(DHmat(4,1))*TZ(DHmat(4,3))*TX(DHmat(4,2))*RX(DHmat(4,4));
T5 = RZ(DHmat(5,1))*TZ(DHmat(5,3))*TX(DHmat(5,2))*RX(DHmat(5,4));
T6 = RZ(DHmat(6,1))*TZ(DHmat(6,3))*TX(DHmat(6,2))*RX(DHmat(6,4));

% Next calculate the final transformation which is the result of
% performing the six separate transformations in succession:
Tfinal = T1*T2*T3*T4*T5*T6;

% Finally, return only the part of the transformation matrix that
% encodes position:
pos = [Tfinal(1,4),Tfinal(2,4),Tfinal(3,4)];

end
