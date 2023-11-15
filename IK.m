function theta = IK(T,theta0)
% Numerical calculation of UR5e inverse kinematics for end-effector
% position described by the transformation matrix T, starting from
% initial guess theta0 for the angles.

% Make sure theta0 is a column vector
if isequal(size(theta0),[1 6])
  theta0 = theta0';
elseif ~isequal(size(theta0),[6 1])
  disp('Initial guess needs to be a 1D vector');
  return
end

% Repeating the arm geometry from the FK lab; all values in mm:
W2 = 259.6;
W1 = 133;
H2 = 99.7;
H1 = 162.5;
L1 = 425;
L2 = 392;

% Joint screw axes in the world frame, vector form:
S1 = [0;0;1;0;0;0];
S2 = [0;1;0;-H1;0;0];
S3 = [0;1;0;-H1;0;L1];
S4 = [0;1;0;-H1;0;L1+L2];
S5 = [0;0;-1;-W1;L1+L2;0];
S6 = [0;1;0;H2-H1;0;L1+L2];

% Transformation between the world and body frames when at home position:
M = [-1,0,0,L1+L2;
      0,0,1,W1+W2;
      0,1,0,H1-H2;
      0,0,0,1];

% Joint screw axes in the world frame, matrix form ([bracketed]):
S1b = joint_screw_mat_form(S1);
S2b = joint_screw_mat_form(S2);
S3b = joint_screw_mat_form(S3);
S4b = joint_screw_mat_form(S4);
S5b = joint_screw_mat_form(S5);
S6b = joint_screw_mat_form(S6);

% Joint screw axes in the body frame, matrix form:
B1b = inv(M)*S1b*M;
B2b = inv(M)*S2b*M;
B3b = inv(M)*S3b*M;
B4b = inv(M)*S4b*M;
B5b = inv(M)*S5b*M;
B6b = inv(M)*S6b*M;

% Joint screw axes in the body frame, vector form:
B1 = [-B1b(2,3),B1b(1,3),-B1b(1,2),B1b(1,4),B1b(2,4),B1b(3,4)]';
B2 = [-B2b(2,3),B2b(1,3),-B2b(1,2),B2b(1,4),B2b(2,4),B2b(3,4)]';
B3 = [-B3b(2,3),B3b(1,3),-B3b(1,2),B3b(1,4),B3b(2,4),B3b(3,4)]';
B4 = [-B4b(2,3),B4b(1,3),-B4b(1,2),B4b(1,4),B4b(2,4),B4b(3,4)]';
B5 = [-B5b(2,3),B5b(1,3),-B5b(1,2),B5b(1,4),B5b(2,4),B5b(3,4)]';
B6 = [-B6b(2,3),B6b(1,3),-B6b(1,2),B6b(1,4),B6b(2,4),B6b(3,4)]';

% Rotation vectors (omega-hat) for the joint screw axes in the body frame, vector form:
w1 = [0;1;0];
w2 = [0;0;1];
w3 = [0;0;1];
w4 = [0;0;1];
w5 = [0;-1;0];
w6 = [0;0;1];

% Rotation vectors (omega-hat) for the joint screw axes in the body frame, matrix form:
w1b = [0,-w1(3),w1(2);w1(3),0,-w1(1);-w1(2),w1(1),0];
w2b = [0,-w2(3),w2(2);w2(3),0,-w2(1);-w2(2),w2(1),0];
w3b = [0,-w3(3),w3(2);w3(3),0,-w3(1);-w3(2),w3(1),0];
w4b = [0,-w4(3),w4(2);w4(3),0,-w4(1);-w4(2),w4(1),0];
w5b = [0,-w5(3),w5(2);w5(3),0,-w5(1);-w5(2),w5(1),0];
w6b = [0,-w6(3),w6(2);w6(3),0,-w6(1);-w6(2),w6(1),0];

% Here begins the iterative algorithm described in the textbook,
% in the form described starting "To modify this algorithm to work
% with a desired end-effector configuration represented as T_sd...":

thguess = theta0;  % initialize the current guess to the user-supplied value
lastguess = thguess * 10 + 50;  % arbitrary value far away from the initial guess, to ensure the while loop is entered

% Termination condition, indicating the algorithm has converged:
while norm(thguess-lastguess) > 1e-3
  norm(thguess-lastguess);

  lastguess = thguess;

% Step (b) of the iterative algorithm is:
% "Set [V_b] = log(T^{-1}_{ab}(theta^i)T_{ad})"
% We can go about that as follows:
  
% From above, we have the joint screw axes in the body frame; now for5
% each joint, convert from the exponential coordinate representation
% for the rotation around the screw axis ((B,theta) form) to the 4x4
% matrix representation of the transformation ((R,d) form):
  eB1 = exp_screw(B1(1:3),B1(4:6),lastguess(1));
  eB2 = exp_screw(B2(1:3),B2(4:6),lastguess(2));
  eB3 = exp_screw(B3(1:3),B3(4:6),lastguess(3));
  eB4 = exp_screw(B4(1:3),B4(4:6),lastguess(4));
  eB5 = exp_screw(B5(1:3),B5(4:6),lastguess(5));
  eB6 = exp_screw(B6(1:3),B6(4:6),lastguess(6));

% To calculate the Jacobian, start by transforming each of the body
% screw axes by all the joints closer to the base, which is easiest to
% do in the matrix form:
  bJ1 = inv(eB6)*inv(eB5)*inv(eB4)*inv(eB3)*inv(eB2)* B1b *eB2*eB3*eB4*eB5*eB6;
  bJ2 = inv(eB6)*inv(eB5)*inv(eB4)*inv(eB3)* B2b *eB3*eB4*eB5*eB6;
  bJ3 = inv(eB6)*inv(eB5)*inv(eB4)* B3b *eB4*eB5*eB6;
  bJ4 = inv(eB6)*inv(eB5)* B4b *eB5*eB6;
  bJ5 = inv(eB6)* B5b *eB6;
  bJ6 = B6b;
  
% Each of the columns of the Jacobian is the vector form of the above:
  J1 = [-bJ1(2,3),bJ1(1,3),-bJ1(1,2),bJ1(1,4),bJ1(2,4),bJ1(3,4)]';
  J2 = [-bJ2(2,3),bJ2(1,3),-bJ2(1,2),bJ2(1,4),bJ2(2,4),bJ2(3,4)]';
  J3 = [-bJ3(2,3),bJ3(1,3),-bJ3(1,2),bJ3(1,4),bJ3(2,4),bJ3(3,4)]';
  J4 = [-bJ4(2,3),bJ4(1,3),-bJ4(1,2),bJ4(1,4),bJ4(2,4),bJ4(3,4)]';
  J5 = [-bJ5(2,3),bJ5(1,3),-bJ5(1,2),bJ5(1,4),bJ5(2,4),bJ5(3,4)]';
  J6 = [-bJ6(2,3),bJ6(1,3),-bJ6(1,2),bJ6(1,4),bJ6(2,4),bJ6(3,4)]';

% Finally we can assemble the complete Jacobian:
  J = [J1, J2, J3, J4, J5, J6];

% Forward kinematics for the robot's pose as a function of theta:
  Tab = M*eB1*eB2*eB3*eB4*eB5*eB6;

% Convert the transformation matrix we want (the input T) into the body frame:
  Tbd = inv(Tab)*T;

% Calculate the matrix logarithm of T_bd:
  pbd = Tbd(1:3,4);
  if norm(Tbd(1:3,1:3)-eye(3)) <= 1e-5
      wbd = [0;0;0];
      wbd_mat = [0,-wbd(3),wbd(2);wbd(3),0,-wbd(1);-wbd(2),wbd(1),0];
      vbd = pbd/norm(pbd);
      theta_bd = norm(pbd);
  else
      if trace(Tbd(1:3,1:3))+1 <= 1e-5
          theta_bd = pi;
          if abs(Tbd(3,3)+1) > 1e-5
              wbd = (1/sqrt(2*(1+Tbd(3,3))))*[Tbd(1,3); Tbd(2,3); 1+Tbd(3,3)];
          elseif abs(Tbd(2,2)+1) > 1e-5
              wbd = (1/sqrt(2*(1+Tbd(2,2))))*[Tbd(1,2); 1+Tbd(2,2); Tbd(3,2)];
          else
              wbd = (1/sqrt(2*(1+Tbd(1,1))))*[1+Tbd(1,1); Tbd(2,1); Tbd(3,1)];
          end
          wbd_mat = [0,-wbd(3),wbd(2);wbd(3),0,-wbd(1);-wbd(2),wbd(1),0];
      else
          theta_bd = acos(0.5*(Tbd(1,1)+Tbd(2,2)+Tbd(3,3)-1));
          wbd_mat = (1/(2*sin(theta_bd)))*(Tbd(1:3,1:3)-(Tbd(1:3,1:3))');
          wbd = [-wbd_mat(2,3),wbd_mat(1,3),-wbd_mat(1,2)];
      end
      G_inv = eye(3)/theta_bd-0.5*wbd_mat+(1/theta_bd-0.5*cot(theta_bd/2))*wbd_mat*wbd_mat;
      vbd = G_inv*pbd;
  end
  

% Finally we can set [Vb] equal to that log, which is what we set out to do:
  Vbb = [wbd_mat*theta_bd, vbd*theta_bd; 0,0,0,0];

% Convert the matrix form of the body twist to the vector form, [Vb] -> Vb:
  Vb = [-Vbb(2,3);Vbb(1,3);-Vbb(1,2);Vbb(1,4);Vbb(2,4);Vbb(3,4)];

% The last step is to update the current guess for the angles, using
% "Set theta^{i+1} = theta^{i} + pinv(Jb(theta^{i}))*Vb":
  thguess = thguess + pinv(J)*Vb;
end

% Once the algorithm has converged, return the final angle found:
theta = mod(thguess,2*pi);
if theta(2) > pi/4
    theta(2) = theta(2)-2*pi;
end
if theta(3) > 3*pi/4
    theta(3) = theta(3)-2*pi;
end

% helper functions
function es = exp_screw(w,v,theta)
% gets a screw matrix and an angle and returns their exponent matrix
w_mat = [0,-w(3),w(2);w(3),0,-w(1);-w(2),w(1),0];
ewthet = eye(3)+sin(theta)*w_mat+(1-cos(theta))*w_mat*w_mat;
g = (eye(3)*theta+(1-cos(theta))*w_mat+(theta-sin(theta))*w_mat*w_mat)*v;
es = zeros(4,4);
es(1:3,1:3) = ewthet;
es(1:3,4) = g;
es(4,4) = 1;
end

function sb  = joint_screw_mat_form(s)
% Joint screw axes in the world frame, matrix form ([bracketed]):
w = s(1:3);
v = s(4:6);
w_mat = [0,-w(3),w(2);w(3),0,-w(1);-w(2),w(1),0];
sb = zeros(4,4);
sb(1:3,1:3) = w_mat;
sb(1:3,4) = v;
end
end











