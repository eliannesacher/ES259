function torques = NE(theta,thetadot,thetadotdot,Ftip)
% Uses the Newton-Euler inverse dynamics algorithm to calculate the torques
% needed to generate the trajectory given by the inputs, for the robot
% described in the lab assignment.
% theta, thetadot, and thetadotdot are three-element vectors, giving the trajectory.
% Ftip is a six-element vector, giving the wrench applied to the environment by the tip.

% Make sure all inputs are column vectors
if isequal(size(theta),[1 3])
  theta = theta';
elseif ~isequal(size(theta),[3 1])
  disp('theta needs to be a 1D vector');
  return
end
if isequal(size(thetadot),[1 3])
  thetadot = thetadot';
elseif ~isequal(size(thetadot),[3 1])
  disp('thetadot needs to be a 1D vector');
  return
end
if isequal(size(thetadotdot),[1 3])
  thetadotdot = thetadotdot';
elseif ~isequal(size(thetadotdot),[3 1])
  disp('thetadotdot needs to be a 1D vector');
  return
end
if isequal(size(Ftip),[1 6])
  Ftip = Ftip';
elseif ~isequal(size(Ftip),[6 1])
  disp('Ftip needs to be a 1D vector');
  return
end

% define values of constants:
L1 = 2;
L2 = 1;
L3 = 0.5;
r = 0.1;
rho = 1;
g = 9.8;

% screw axes of joint i expressed in {i}:
omega1 = [0,0,1]';
q1 = [-L1/2,0,0]';
v1 = -cross(omega1,q1);
A1 = [omega1;v1];


omega2 = [0,-1,0]';
q2 = [0,0,L2/2]';
v2 = -cross(omega2,q2);
A2 = [omega2;v2];

omega3 = [1,0,0]';
q3 = [-L3/2,0,0]';
v3 = -cross(omega3,q3);
A3 = [omega3;v3];

% configuration of {i-1} in {i} at home position:
M10 = [[eye(3),[-L1/2;0;0]];[0,0,0,1]];
M21 = [[eye(3),[-L1/2;0;L2/2]];[0,0,0,1]];
M32 = [[eye(3),[-L3/2;0;L2/2]];[0,0,0,1]];
M43 = [[eye(3),[-L3/2;0;0]];[0,0,0,1]];

% spatial inertia matrices:
m1 = rho*pi*r^2*L1;
I1 = [m1*(3*r^2+L1^2)/12,0,0;0,m1*(3*r^2+L1^2)/12,0;0,0,m1*r^2/2];
G1 = [[I1,zeros(3,3)];[zeros(3,3),eye(3)*m1]];

m2 = rho*pi*r^2*L2;
I2 = [m2*(3*r^2+L2^2)/12,0,0;0,m2*(3*r^2+L2^2)/12,0;0,0,m2*r^2/2];
G2 = [[I2,zeros(3,3)];[zeros(3,3),eye(3)*m2]];

m3 = rho*pi*r^2*L3;
I3 = [m3*(3*r^2+L3^2)/12,0,0;0,m3*(3*r^2+L3^2)/12,0;0,0,m3*r^2/2];
G3 = [[I3,zeros(3,3)];[zeros(3,3),eye(3)*m3]];

% gravity:
V0dot = [0 0 0 0 0 g]';
% assume the base is not moving:
V0 = zeros(6,1);

% calculate exp(-[A_i]theta_i):
omega1_bra = -prod_brack_w(omega1);
expomega1 = exp_from_omegathet(theta(1),omega1_bra);
g1 = -g_term(theta(1),omega1_bra,v1);
expAtheta1 = [[expomega1,g1];[0,0,0,1]];

omega2_bra = -prod_brack_w(omega2);
expomega2 = exp_from_omegathet(theta(2),omega2_bra);
g2 = -g_term(theta(2),omega2_bra,v2);
expAtheta2 = [[expomega2,g2];[0,0,0,1]];

omega3_bra = -prod_brack_w(omega3);
expomega3 = exp_from_omegathet(theta(3),omega3_bra);
g3 = -g_term(theta(3),omega3_bra,v3);
expAtheta3 = [[expomega3,g3];[0,0,0,1]];

% -- Forward iterations --

% calculate the quantities of Equations 8.50--8.52 for all joints:
T10 = expAtheta1*M10;
AdT10 = my_adjoint_for_T(T10);
V1 = AdT10*V0 + A1*thetadot(1);
adV1 = my_adjoint_for_V(prod_brack_V(V1)); 
V1dot = AdT10*V0dot+adV1*A1*thetadot(1)+A1*thetadotdot(1);

T21 = expAtheta2*M21;
AdT21 = my_adjoint_for_T(T21);
V2 = AdT21*V1 + A2*thetadot(2);
adV2 = my_adjoint_for_V(prod_brack_V(V2));
V2dot = AdT21*V1dot+adV2*A2*thetadot(2)+A2*thetadotdot(2);

T32 = expAtheta3*M32;
AdT32 = my_adjoint_for_T(T32);
V3 = AdT32*V2 + A3*thetadot(3);
adV3 = my_adjoint_for_V(prod_brack_V(V3));
V3dot = AdT32*V2dot+adV3*A3*thetadot(3)+A3*thetadotdot(3);

% -- Backward iterations --

% first calculate the last adjoint matrix we'll need:
T43 = M43;
AdT43 = my_adjoint_for_T(T43);

% now calculate the quantities of Equations 8.53--8.54 for all joints:
F3 = (AdT43')*Ftip+G3*V3dot-(adV3')*G3*V3;
tau3 = (F3')*A3;

F2 = (AdT32')*F3+G2*V2dot-(adV2')*G2*V2;
tau2 = (F2')*A2;

F1 = (AdT21')*F2+G1*V1dot-(adV1')*G1*V1;
tau1 = (F1')*A1;

% return the vector of joint torques:
torques = [tau1 tau2 tau3]';

% HELPER FUNCTIONS
function omega_bra = prod_brack_w(omega)
% produces matrix bracket form for omega
omega_bra = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
end

function V_bra = prod_brack_V(V)
% produces matrix bracket form for V
w = V(1:3);
w_brack = prod_brack_w(w);
v = V(4:6);
V_bra = [[w_brack,v];[0,0,0,0]];
end

function [ad] = my_adjoint_for_V(V)
% Produces an adjoint for the bracketed form of V    
R = V(1:3,1:3);
p = V(1:3,4);
p_brack = prod_brack_w(p);
ad = [[R,zeros(3,3)];[p_brack,R]];
end

function [ad] = my_adjoint_for_T(T)
% Produces an adjoint for the bracketed form of T   
R = T(1:3,1:3);
p = T(1:3,4);
p_brack = prod_brack_w(p);
pR = p_brack*R;
ad = [[R,zeros(3,3)];[pR,R]];
end

function g = g_term(theta,omega,v)
% produce the G term matrix
g = (eye(3)*theta + (1-cos(theta))*omega + (theta-sin(theta))*omega*omega)*v;
end

function expomega = exp_from_omegathet(theta,omega)
% gets the value of the exponential of bracketed omega and theta
expomega = eye(3)+sin(theta)*omega+(1-cos(theta))*omega*omega;
end

end
