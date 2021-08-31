###############################################################################
# Illinois Space Society - IREC 2021 Avionics Team
#
# Kalman Filter derivation for full state estimation (now in python!)
# Created 07/31/2021
# 
# Authors:
# Ayberk Yaraneri
#
###############################################################################

import numpy as np
import sympy as sym
import matplotlib.pyplot as plt
from scipy import linalg

from utils import *

# Suppress the use of scientific notation when printing small numbers
np.set_printoptions(suppress=True)

###############################################################################
# Known Physical Constants

dCP = 0.20          # distance between CG and CP
rho = 1.225         # air density
Sref = 0.0232       # reference area 
lref = 4.572        # reference length
m = 27.216          # mass 
Ixx, Iyy, Izz = 1.0, 1.0, 0.01    # moments of inertia (random numbers rn) 

# Aerodynamic moment coefficients. Probably can set to 0 to simplify
C_lpsi, C_malpha, C_mtheta, C_nbeta, C_nphi = 0, 0, 0, 0, 0

###############################################################################
# Symoblic Variables 

# Position and derivatives in NED frame
x, y, z, xdot, ydot, zdot, xddot, yddot, zddot = \
    sym.symbols('x,y,z,xdot,ydot,zdot,xddot,yddot,zddot', real=True)

# Euler angles and derivatives in BODY frame [yaw, pitch, roll]
phi, theta, psi, phidot, thetadot, psidot, phiddot, thetaddot, psiddot = \
    sym.symbols('phi,theta,psi,phidot,thetadot,psidot, \
                 phiddot,thetaddot,psiddot', real=True)

# NED-to-BODY conversion quaternion components and derivatives                               
q1, q2, q3, q4, q1dot, q2dot, q3dot, q4dot = \
    sym.symbols('q1,q2,q3,q4,q1dot,q2dot,q3dot,q4dot', real=True)

# Nominal thrust and thrust scale coefficient
T, nu_T = sym.symbols('T,nu_T', real=True) 

# Axial aerodynamic force coefficents (Y and Z depend on beta and alpha)
nu_Cx, Cx, C_Ybeta, C_Zalpha = \
    sym.symbols('nu_Cx,Cx,C_Ybeta,C_Zalpha', real=True)

C_l0 = sym.symbols('C_l0', real=True) # hmmmm

###############################################################################
# Constructing Useful Vectors

r = sym.Matrix([x, y, z])                   # NED frame position vector
rdot = sym.Matrix([xdot, ydot, zdot])       # NED frame position derivative (velocity)
rddot = sym.Matrix([xddot, yddot, zddot])   # NED frame position 2nd derivative (acceleration)

q = sym.Matrix([q1, q2, q3, q4])                # NED-to-BODY conversion quaternion 
qdot = sym.Matrix([q1dot, q2dot, q3dot, q4dot]) # Derivative of ^^

omega = sym.Matrix([psidot, thetadot, phidot])          # Instantaneous angular velocity vector
oemgadot = sym.Matrix([psiddot, thetaddot, phiddot])    # Instantaneous angular acceleration vector

CP_vect = sym.Matrix([-dCP, 0, 0])     # BDY frame vector pointing from CG to CP

###############################################################################
# Constructing Forces and Monents + Axial Acceleration

# Velocity (performing NED->BDY rotation)
V_BDY = VecRotateQuat(rdot, q);
vx, vy, vz = V_BDY[0:3]

# Acceleration (performing NED->BDY rotation)
A_BDY = VecRotateQuat(rddot, q);
ax, ay, az = A_BDY[0:3]

# Velocity magnitudes for convenience
V = sym.sqrt(V_BDY.dot(V_BDY))
V2 = V_BDY.dot(V_BDY)

# Acceleration due to gravity (performing NED->BDY rotation)
accel_grav_NED = sym.Matrix([0, 0, 9.81])
accel_grav_BDY = VecRotateQuat(accel_grav_NED, q)

# Acceleration due to thrust (performing BDY->NED rotation)
accel_thrust = (nu_T * T / m)
accel_thrust_BDY = sym.Matrix([accel_thrust, 0, 0])
accel_thrust_NED = VecRotateQuatInv(accel_thrust_BDY, q)

# Angle of attack alpha and side-slip angle beta
alpha = sym.atan2(vz, vx);
beta = sym.atan2(vy, sym.sqrt(vz**2 + vx**2))

# Aerodynamic force acting at the location of CP in BDY frame
force_aero_BDY = sym.Matrix([(nu_Cx*Cx), \
                             (C_Ybeta*beta), \
                             (C_Zalpha*alpha)]) * (0.5*rho*V2*Sref)
fx, fy, fz = force_aero_BDY[0:3]

# Acceleration due to aero forces (performing BDY->NED rotation)
accel_aero_BDY = (1/m) * force_aero_BDY
accel_aero_NED = VecRotateQuatInv(accel_aero_BDY, q)

# Aerodynamic moment in BDY frame
moment_aero_BDY = CP_vect.cross(force_aero_BDY) + \
                  sym.Matrix([ C_l0 + C_lpsi*(psi*lref/(2*V)),               \
                               C_malpha*alpha + C_mtheta*(theta*lref/(2*V)), \
                               C_nbeta*beta + C_nphi*(phi*lref/(2*V))        ]) \
                               * 0.5*rho*V2*Sref*lref;
mx, my, mz = moment_aero_BDY[0:3]
                               
###############################################################################
# Constructing State Transtion Functions (i.e. the state derivative function f)

# State transition function of NED position and NED velocity
r_func = rdot
rdot_func = accel_grav_NED + accel_thrust_NED + accel_aero_NED

# State transition function of orientation quaterion
q_func = 0.5 * QuatMult(q, sym.Matrix([[omega], [0]]))

# State transition function of angular velocity vector (BDY frame)
omega_func = sym.Matrix([ (mx + (Iyy-Izz)*theta*phi)/Ixx,
                          (my + (Izz-Ixx)*phi*theta)/Iyy,
                          (mz + (Ixx-Iyy)*psi*theta)/Izz])

# TOTAL state transition function f:
f = sym.Matrix([ r_func     ,
                 rdot_func  ,
                 q_func     ,
                 omega_func ,
                 sym.S(0)   ,  # don't know this states transition function yet
                 sym.S(0)   ,  # don't know this states transition function yet
                 sym.S(0)   ]) # don't know this states transition function yet

###############################################################################
# Linearization

states = [x,y,z,xdot,ydot,zdot,q1,q2,q3,q4,psidot,thetadot,phidot,nu_T,nu_Cx,C_l0]

# Equilibrium states
x_e, y_e, z_e = 0, 0, 0
xdot_e, ydot_e, zdot_e = 0, 0, 100

q_eqb = EulToQuat(sym.Matrix([0, sym.pi/2, 0]))
q1_e = float(q_eqb[0]) 
q2_e = float(q_eqb[1])
q3_e = float(q_eqb[2])
q4_e = float(q_eqb[3])

psidot_e, thetadot_e, phidot_e = 0, 0, 0

nu_T_e = 0.0
nu_Cx_e = 0.0
C_l0_e = 0.0

A_lambda = sym.lambdify(states, f.jacobian(states))

A = A_lambda(x_e, y_e, z_e, xdot_e, ydot_e, zdot_e, \
          q1_e, q2_e, q3_e, q4_e, psidot_e, thetadot_e, phidot_e, nu_T_e, nu_Cx_e, C_l0_e)

print(A)


