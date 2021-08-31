%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Illinois Space Society - IREC 2021 Avionics Team
%
% Kalman Filter derivation for full state estimation
% Created 05/02/2021
% 
% Authors:
% Ayberk Yaraneri
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%

% Position and derivatives in NED frame
syms x y z xdot ydot zdot xddot yddot zddot 

% Euler angles and derivatives in BODY frame [yaw, pitch, roll]
syms phi theta psi phidot thetadot psidot phiddot thetaddot psiddot
                                         
% NED-to-BODY conversion quaternion components and derivatives                               
syms q1 q2 q3 q4 q1dot q2dot q3dot q4dot 

% Mass and moments of inertia
syms m Ixx Iyy Izz 

% Nominal thrust and thrust scale coefficient
syms T nu_T 

% Axial aerodynamic force coefficents (Y and Z depend on beta and alpha)
syms nu_Cx Cx C_Ybeta C_Zalpha

% Air density, reference area, reference length
syms rho Sref lref

% Distance between CG and CP
syms dCP

% Aerodynamic moment coefficients. Probably can set to 0 to simplify
syms C_l0 C_lpsi C_malpha C_mtheta C_nbeta C_nphi
                                         
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constructing useful vectors                                       
                                         
r = [x ; y ; z];                 % NED frame position vector
rdot = [xdot ; ydot ; zdot];     % NED frame position derivative (velocity)
rddot = [xddot ; yddot ; zddot]; % NED frame position 2nd derivative (acceleration)

q = [q1 ; q2 ; q3 ; q4];                % NED-to-BODY conversion quaternion
qdot = [q1dot ; q2dot ; q3dot ; q4dot]; % Derivative of ^^

omega = [psidot ; thetadot ; phidot];       % Instantaneous angular velocity vector
oemgadot = [psiddot ; thetaddot ; phiddot]; % Instantaneous angular acceleration


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constructing forces and moments + axial acceleration components

% Velocity and acceleration (performing NED->BDY rotation)
V_BDY = VecRotateQuat(rdot, q);
vx = V_BDY(1); vy = V_BDY(2); vz = V_BDY(3);
A_BDY = VecRotateQuat(rddot, q);
ax = A_BDY(1); ay = A_BDY(2); az = A_BDY(3);

% Velocity magnitude for convenience
V = norm(rdot);
V2 = norm(rdot)^2;

% Acceleration due to gravity (performing NED->BDY rotation)
aG_NED = [0 ; 0 ; 9.81];
aG_BDY = VecRotateQuat(aG_NED, q);

% Acceleration due to thrust (performing BDY->NED rotation)
aT_BDY = [(nu_T * T / m) ; 0 ; 0];
aT_NED = VecRotateQuatInv(aT_BDY, q);

% Angle of attack alpha and side-slip angle beta
alpha = atan2(vz, vx);
beta = atan2(vy, sqrt(vz^2 + vx^2));

% Aerodynmaic forces acting at the location of CP in BDY frame
F_Aero_BDY = [nu_Cx*Cx ; C_Ybeta*beta ; C_Zalpha*alpha] * 0.5*rho*V2*Sref;

% Acceleration due to aero forces (performing BDY->NED rotation)
aA_BDY = (1/m) * F_Aero_BDY;
aA_NED = VecRotateQuatInv(aA_BDY, q);

% Aerodynamic moment in BDY frame
M_BDY = cross(dCP * [-1 ; 0 ; 0], F_Aero_BDY) + ...
        [ C_l0 + C_lpsi*(psi*lref/(2*V))               ;
          C_malpha*alpha + C_mtheta*(theta*lref/(2*V)) ; 
          C_nbeta*beta + C_nphi*(phi*lref/(2*V))       ] * 0.5*rho*V2*Sref*lref;
      
Mx = M_BDY(1); My = M_BDY(2); Mz = M_BDY(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State transition functions (i.e. the state derivative function f)

% State transition function of NED position and NED velocity
r_func = rdot;
rdot_func = aG_NED + aT_NED + aA_NED;

% State transition function of orientation quaterion
q_func = 0.5 * QuatMult(q, [omega ; 0]);

% State transition function of angular velocity vector (BDY frame)
omega_func = [ (Mx + (Iyy-Izz)*theta*phi)/Ixx ;
               (My + (Izz-Ixx)*phi*theta)/Iyy ;
               (Mz + (Ixx-Iyy)*psi*theta)/Izz ];

% TOTAL state transition function f:
f = [ r_func     ;
      rdot_func  ;
      q_func     ;
      omega_func ;
      0          ;
      0          ;
	  0          ];

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Linearization

% Equilibrium states to linearize about
r_eqb = [0; 0; 0];
rdot_eqb = [0 ; 0 ; -100];
q_eqb = EulToQuat([0 ; pi/2 ; 0]); % Vehicle oriented directly upwards
omega_eqb = [0 ; 0 ; 0];
nu_T_eqb = 1500.0;
nu_Cx_eqb = 1.0;
C_l0_eqb = 1.0;

eqb_states = [r_eqb ; rdot_eqb ; q_eqb ; omega_eqb ; nu_T_eqb ; ...
              nu_Cx_eqb ; C_l0_eqb];

states = [r ; rdot ; q ; omega ; nu_T ; nu_Cx ; C_l0 ];

A = subs(jacobian(f, states), states, eqb_states);

%%

function quatOut = EulToQuat(Euler)
% Convert from a 321 Euler rotation sequence specified in radians to a Quaternion

Euler    = Euler * 0.5;
cosPhi   = cos(Euler(1));
sinPhi   = sin(Euler(1));
cosTheta = cos(Euler(2));
sinTheta = sin(Euler(2));
cosPsi   = cos(Euler(3));
sinPsi   = sin(Euler(3));

quatOut = [ sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi ;
            cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi ;
            cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi ;
            cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi ];

end

% MUST BE SWITCHED TO [complex, real] QUATERNION NOTATION BEFORE USE
% function Euler = QuatToEul(quat)
% % Convert from a quaternion to a 321 Euler rotation sequence in radians
% 
% Euler = zeros(3,1);
% 
% Euler(1) = atan2(2*(quat(3)*quat(4)+quat(1)*quat(2)),  quat(1)*quat(1) - quat(2)*quat(2) - quat(3)*quat(3) + quat(4)*quat(4));
% Euler(2) = -asin(2*(quat(2)*quat(4)-quat(1)*quat(3)));
% Euler(3) = atan2(2*(quat(2)*quat(3)+quat(1)*quat(4)),  quat(1)*quat(1) + quat(2)*quat(2) - quat(3)*quat(3) - quat(4)*quat(4));
% 
% end

function quatOut = QuatMult(qA,qB)
% Perform a quaternion multiplication

quatOut = [ qA(4),-qA(3), qA(2), qA(1) ;
            qA(3), qA(4),-qA(1), qA(2) ;
           -qA(2), qA(1), qA(4), qA(3) ;
           -qA(1),-qA(2),-qA(3), qA(4) ] * qB;

end

function vecOut = VecRotateQuat(v, q)
% Rotate vector v using quaternion q

qInv = [-q(1) ; -q(2) ; -q(3) ; q(4)];

quatOut = QuatMult(QuatMult(q, [v ; 0]), qInv);
vecOut = quatOut(1:3);

end

function vecOut = VecRotateQuatInv(v, q)
% Rotate vector v using inverse of quaternion q

qInv = [-q(1) ; -q(2) ; -q(3) ; q(4)];

quatOut = QuatMult(QuatMult(qInv, [v ; 0]), q);
vecOut = quatOut(1:3);

end

