#! /usr/bin/env python
import rospy
import numpy as np
import math
import numpy.matlib
from numpy import linalg as LA
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from scipy.integrate import odeint
from scipy.optimize import minimize
from casadi import *
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow import keras
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation
# from keras.optimizers import SGD
from pytictoc import TicToc
tt = TicToc()

#---> ####### GLOBAL VARIABLES ####### <---#
d_max = 1.5 # 3.5
# rob_diam = 0.2
numRays = 72
scan = d_max*np.ones((1,numRays))
Angles = np.linspace(0, 2*np.pi, num=numRays)
# Angles = np.concatenate((np.linspace(0, 1*np.pi, num=36), np.linspace(0, -1*np.pi, num=36)), axis=0)
# number of lidar rays
ns = 200
# ns number of samples on each ray including safe and unsafe samples
Ds = np.linspace(0, d_max, num=ns)
# bound is the shield between the measured distance by lidar and the robot in order to:
# (1) have more data on boundary of obstacles
# (2) add some threshold to address uncertain lidar measurements
bound = 0.15 # 0.05
# Angles = np.concatenate((np.linspace(0.0, np.pi, 180), np.linspace(-np.pi, 0.0, 180)), axis=0)
# print("Angles: ", Angles)
lc = 0.0									                                    # unicycle model correction constant
T = 0.05 # 0.025                                                                         # [s]
N = 20 # 15                                                                          # prediction horizon
M = int(N/2)
Xr = np.array([[0.0],[0.0],[0.0]])
# Xr = np.array([[0.0],[0.0],[0.0]])
# Xr = np.array([[-3.0],[1.0],[0.0]])

#---> ####### Sensors ######## <---#
def callback_odom(odom):
    global Xr
    xr = odom.pose.pose.position.x
    yr = odom.pose.pose.position.y
    qz = odom.pose.pose.orientation.z
    qw = odom.pose.pose.orientation.w
    thr = 2*np.arcsin(qz)
    Xr = np.array([[xr], [yr], [thr]])


def callback_lidar(lidar):
    global scan # min_dist_scan, min_dist_ang, max_dist
    scan = list(lidar.ranges)
    for i in range(len(scan)):
        # scan[i] = scan[i] - rob_diam
        if scan[i] == np.Inf:
           scan[i] = d_max


#---> ####### MPC ######## <---#
def shift(T, t0, u):
    # st = x0
    con = np.transpose(u[0:1,0:])
    # f_value = f(st, con)
    # st = st + (T*f_value)
    # x0 = st.full()
    t0 = t0 + T
    ushape = np.shape(u)
    u0 = np.concatenate(( u[1:ushape[0],0:],  u[ushape[0]-1:ushape[0],0:]), axis=0)
    return t0, u0


# ===============================================================================
def safe_boundary_unsafe_sample_creator(d_lidar, Xr, ns):
    # print("DS: ", Ds[0])
    # print("scan: ", np.shape(scan))
    # d_l = np.reshape(d_lidar, (1, -1))
    Ds = np.linspace(0, d_max, num=ns)
    inputs = [[],[]]
    output = []
    gRl = np.array([ [np.cos(Xr[2,0]),  -np.sin(Xr[2,0])], [np.sin(Xr[2,0]),  np.cos(Xr[2,0])] ])
    for i in range(numRays):
        for s in range(ns):
            if (Ds[s] < d_lidar[i]-bound) and (Ds[s] > d_lidar[i]+bound):
                # safe samples
                d_local = Ds[s]*np.array([[np.cos(Angles[i])], [np.sin(Angles[i])]])
                xy = Xr[0:2] + np.matmul(gRl, d_local)
                inputs = np.append(inputs, xy, axis=1)
                output = np.append(output, [d_lidar[i]-bound-Ds[s]], axis=0)
                #
            elif (Ds[s] > d_lidar[i]-bound) and (Ds[s] < d_lidar[i]+bound):
                # unsafe samples
                d_local = Ds[s]*np.array([[np.cos(Angles[i])], [np.sin(Angles[i])]])
                xy = Xr[0:2] + np.matmul(gRl, d_local)
                inputs = np.append(inputs, xy, axis=1)
                output = np.append(output, [d_lidar[i]-bound-Ds[s]], axis=0)

    return np.transpose(inputs), np.reshape(output, (-1,1))

def tanH(Z):
    # print("Z", Z)
    A = tanh(Z)
    return A


def FP_BP(Xk, WB_sym, numLayers):
    Al_1 = Xk
    dh_dX = 1
    for l in range(numLayers):
        # FP
        wl = WB_sym[int(2*l)]
        bl = reshape(WB_sym[int(2*l+1)], -1, 1)
        Zl = mtimes(wl.T, Al_1) + bl
        Al = tanH(Zl)
        # BP
        dAl_dZl = 1 - Al**2
        dZl_dAl_1 = wl.T
        dAl_dAl_1 = dAl_dZl*dZl_dAl_1
        # Next layer
        Al_1 = Al
        # print("====> ", np.shape(dAl_dAl_1), np.shape(dh_dX))
        dh_dX = mtimes(dAl_dAl_1, dh_dX)

    h = Al
    hdot = dh_dX
    return h, dh_dX


def list2flat_WB(Weights_Biases):
    WB = []
    for i in range(len(Weights_Biases)):
        # print("weights_biases[i].flatten('F'): ", np.shape(weights_biases[i].flatten('F')))
        wb = np.reshape(Weights_Biases[i].flatten('F'), (1,-1))
        # print("wb: ", np.shape(wb))
        WB = np.append(WB, wb)

    # print("WB: ", np.shape(WB))
    return np.reshape(WB, (-1,1))


#---> ####### node initialization & subscribers ######## <---#
rospy.init_node('move_node_1')
# sub1 = rospy.Subscriber('/odom', Odometry, callback_odom)
sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, callback_odom)
# sub2 = rospy.Subscriber('/scan', LaserScan, callback_lidar)
sub2 = rospy.Subscriber('/tb3_1/scan', LaserScan, callback_lidar)


#===============================================================================
# Create the model
model = keras.Sequential()
# model.add(keras.layers.Dense(units = 1, activation = 'linear', input_shape=(36000) ))
# model.add(Dense(10, input_dim=2, activation="relu"))
model.add(keras.layers.Dense(units = 32, activation = 'tanh', input_shape=(2,)))
model.add(keras.layers.Dense(units = 32, activation = 'tanh'))
model.add(keras.layers.Dense(units = 16, activation = 'tanh'))
model.add(keras.layers.Dense(units = 8, activation = 'tanh'))
model.add(keras.layers.Dense(units = 1, activation = 'tanh'))
#
opt = keras.optimizers.Adam(lr=0.01)
model.compile(loss='mse', optimizer=opt)
model.summary()
numLayers = len(model.layers)
#

Inputs, Output = safe_boundary_unsafe_sample_creator(scan, Xr, ns)
# print("shape inp: ", Inputs[:,0])
# print("shape out: ", Output[:,0])


# Training
tt.tic()
model.fit(Inputs, Output, epochs=5000, verbose=0, batch_size=36**10)
# model.fit(x_data, y_data, epochs=500, verbose=1)
tt.toc()
#
Init_Weights_Biases = model.get_weights() # returs a numpy list of weights and biases
y_pred = model.predict(Inputs)
print("mse: ", np.linalg.norm(Output - y_pred)/len(y_pred))
#----------------------------------------------------------
numW = []; numB = []
for l in range(len(Init_Weights_Biases)):
    if l%2 ==0:
        # means that weights[l] shows weights on layer l of NN
        numW = np.append(numW,np.size(Init_Weights_Biases[l]))
    elif l%2 != 0:
        # means that weights[l] shows biases on layer l of NN
        # print("bias: ", weights[l])
        numB = np.append(numB,np.size(Init_Weights_Biases[l]))

# print("num of weights: ", numW)
# print("num of neurons: ", numB)
#----------------------------------------------------------

#---> ####### Problem Setting ######## <---#
v_max = +0.2; v_min = -v_max
omega_max = +0.75; omega_min = -omega_max
x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta')
states = np.array([[x], [y], [theta]]); n_states = len(states)
# print("states: ", states)

#---> ####### MPC Setting ########### <---#
v = SX.sym('v'); omega = SX.sym('omega')
controls = np.array([[v],[omega]]); n_controls = len(controls)
rhs = np.array([[v*np.cos(theta)],[v*np.sin(theta)],[omega]])                   # system r.h.s
# print("rhs: ", rhs)
f = Function('f',[states,controls],[rhs])                                       # nonlinear mapping function f(x,u)
# print("Function :", f)

total_num_WB = int(np.sum(numW)) + int(np.sum(numB))
# print("total num W: ", total_numWeights)
U = SX.sym('U',n_controls,N);                                                   # Decision variables (controls)
P = SX.sym('P',n_states + n_states + total_num_WB)              # parameters (which include the initial state, reference state, and number of NN weights)
# print("U: ", U)
# print("P: ", P)

X = SX.sym('X',n_states,(N+1));
# A vector that represents the states over the optimization problem.
#print("X: ", X)

obj = 0                                                                        # Objective function                                                                        # constraints vector
Q = np.zeros((3,3)); Q[0,0] = 2.5; Q[1,1] = 2.5; Q[2,2] = 0.05                           # weighing matrices (states) Original weights Q: 1, 5, 0.1
QF = np.zeros((3,3)); Q[0,0] = 5; Q[1,1] = 5; Q[2,2] = 0.05                           # weighing matrices (states) Original weights Q: 1, 5, 0.1
R = np.zeros((2,2)); R[0,0] = 0.05; R[1,1] = 0.05                                  # weighing matrices (controls) Original weights R: 0.5, 0.05
#print("Q: ", Q)
#print("R: ", R)

#---> ####### OBJECTIVE FUNCTION & CONSTRAINTS ####### <---#
st  = X[:,0]                                                                    # initial state
#print("st: ", st)
g = st-P[0:3]                                                                   # initial condition constraints
#print("g: ", g)
# min_dist = P[6]
for k in range(N):
    st = X[:,k];  con = U[:,k]
    #print("st: ", st); print("con: ", con)
    #print("R: ", R)
    obj = obj + mtimes((st-P[3:6]).T,mtimes(Q,(st-P[3:6]))) + mtimes(con.T, mtimes(R, con))               # calculate obj
    #print("Obj: ", obj)
    st_next = X[:,k+1];
    #print("st_next: ", st_next)
    f_value = f(st,con);
    #print("f_value: ", f_value)
    st_next_euler = st + (T*f_value)
    #print("st_next_euler: ", st_next_euler)
    g = vertcat(g, st_next-st_next_euler)                                               # compute constraints


# terminal cost
obj = obj  +  mtimes((X[:,N]-P[3:6]).T,mtimes(QF,(X[:,N]-P[3:6])))

#---> ####### CONTROL BARRIER CONSTRAINTS ######## <---
# Flat2List of WB
WB_sym = []
nL_1 = 2 # number of inputs
start_idx_W = 6
for l in range(numLayers):
    n_W = int(numW[l]); # print("n_W: ", n_W)
    n_B = int(numB[l]); # print("n_W: ", n_B)
    n_L = n_B
    end_idx_W = start_idx_W + n_W
    W = reshape(P[start_idx_W:end_idx_W], nL_1, n_L)
    start_idx_B = end_idx_W
    end_idx_B = start_idx_B + n_B
    # print("W: ", W)
    WB_sym.append(W)
    B = reshape(P[start_idx_B:end_idx_B], -1, 1)
    start_idx_W = end_idx_B
    # print("B: ", B)
    WB_sym.append(B)
    nL_1 = n_L

# print("WB: ", WB_sym)


# print(WB_sym)
CBC = []
for k in range(M):
    #
    h, hdot = FP_BP(X[0:2,k], WB_sym, numLayers)
    # print("===> ", np.shape(h), np.shape(hdot))
    # gRl is rotation matrix from local to global frame
    # h = FP(X[0:2,k], WB_sym, numLayers)
    # print("h: ", h)
    # delta = 0.00001
    # dx = np.array([[delta],[0.0]])
    # dy = np.array([[0.0],[delta]])
    # hdot_dx = (FP(X[0:2,k] + dx, WB_sym, numLayers) - FP(X[0:2,k] - dx, WB_sym, numLayers))/(2*delta)
    # hdot_dy = (FP(X[0:2,k] + dy, WB_sym, numLayers) - FP(X[0:2,k] - dy, WB_sym, numLayers))/(2*delta)
    # hdot = np.array([[hdot_dx], [hdot_dy]])
    # hdot = np.array([[hdot[0]], [hdot[1]], [0.0]])
    # print("hdot: ", hdot)
    # Control Barrier Condition
    gx = np.array([[np.cos(X[2,k]), 0.0], [np.sin(X[2,k]), 0.0]])
    # modified unicycle model
    # gx = np.array([[np.cos(X[2,k]), -lc*np.sin(X[2,k])], [np.sin(X[2,k]), lc*np.cos(X[2,k])], [0.0, 1.0]])
    # cbc = mtimes(hdot.T, mtimes(gx, U[:,k])) + 0.5*BF
    # cbc = mtimes(hdot.T, mtimes(gx, U[:,k])) + 0.1*h
    # print("==> ", np.shape( mtimes(hdot, mtimes(gx, U[:,k]))))
    cbc = mtimes(hdot, mtimes(gx, U[:,k])) + 0.1*h
    # cbc = mtimes(hdot.T, mtimes(gx, U[:,k])) + 0.1*tan(h)
    # print("cbc: ", np.shape(cbc))
    # cbc = mtimes(hdot.T, mtimes(gx, U[:,k])) + 1/np.log(1+BF)
    CBC = vertcat(CBC, cbc)


g = vertcat(g, CBC)
# print("g: ", g)


#---> ####### Minimization Problem ######## <---#
# make the decision variable one column  vector
OPT_variables = vertcat(reshape(X, 3*(N+1),1), reshape(U, 2*N, 1))
# print("OPT: ", OPT_variables)
nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}
# print("NLP: ", nlp_prob)

# opts = {'print_time': 0, 'ipopt':{'max_iter':2000, 'print_level':0, 'acceptable_tol':1e-8, 'acceptable_obj_change_tol':1e-6}}
opts = {'print_time': 0, 'ipopt':{'max_iter':100, 'print_level':0, 'acceptable_tol':1e-8, 'acceptable_obj_change_tol':1e-6}}
# opts = {'print_time': 0, 'ipopt':{'max_iter':300, 'print_level':0, 'acceptable_tol':1e-8, 'acceptable_obj_change_tol':1e-6}}
solver = nlpsol('solver','ipopt', nlp_prob, opts)

# encoded cbc
LBG_args = (np.zeros((1,3*(N+1))), np.matlib.repmat(np.array([0.0]),1,M))
UBG_args = (np.zeros((1,3*(N+1))), np.matlib.repmat(np.array([+np.Inf]),1,M))
LBG = np.concatenate(LBG_args, axis=1)
UBG = np.concatenate(UBG_args, axis=1)
LBX_args = (np.matlib.repmat(np.array([[-10.0],[-10.0],[-2*np.pi]]),N+1,1), np.matlib.repmat(np.array([[v_min],[omega_min]]),N,1))
UBX_args = (np.matlib.repmat(np.array([[+10.0],[+10.0],[+2*np.pi]]),N+1,1), np.matlib.repmat(np.array([[v_max],[omega_max]]),N,1))
LBX = np.concatenate(LBX_args, axis=0)
UBX = np.concatenate(UBX_args, axis=0)

# print("LBX: ", LBX)
# print("UBX: ", UBX)

args = {'lbg': LBG, 'ubg': UBG, 'lbx': LBX, 'ubx': UBX}
# print("args: ", args)

sim_tim = 100
t0 = 0
x0 = np.array([[0.0], [0.0], [0.0]])                                                             # initial condition.

# print("x0: ", x0)
# xs = np.array([[+1.0], [+1.5], [0.0]])                                                        # Reference posture.

xx = np.zeros((n_states, int(sim_tim/T)))
# print("xx: ", xx[:,0:1])
xx[:,0:1] = x0                                                                    # xx contains the history of states
t = np.zeros(int(sim_tim/T))
# print("t: ", np.shape(t))
t[0] = t0

u0 = np.zeros((n_controls,N));                                                             # two control inputs for each robot
# print("u0: ", u0)
X0 = np.transpose(repmat(x0,1,N+1))                                                         # initialization of the states decision variables
# print("X0", X0)
cbc0 = np.zeros((1,N));

                                                                   # Maximum simulation time
#
mpciter = 0
xx1 = np.zeros((N+1,n_states,int(sim_tim/T)))
u_cl = np.zeros((int(sim_tim/T),n_controls))

#---
# the main simulaton loop... it works as long as the error is greater
# than 10^-6 and the number of mpc steps is less than its maximum
# value.
# main_loop = tic;
goal_idx = 1
ng = 6

Xr = np.array([[0.0],[0.0],[0.0]])

#---> ####### MPC-CBF-NN Main Loop ######## <---#
while (not rospy.is_shutdown()):
    #
    # READ LIDAR SCAN REAL-TIME TO RETRAIN NN AND UPDATE WEIGHTS
    # TO improve learning >> we can use trained weights in previous step as initial guess for new training
    ns = 50
    Inputs, Output = safe_boundary_unsafe_sample_creator(scan, Xr, ns)
    #
    # tt.tic()
    model.fit(Inputs, Output, epochs=30, verbose=0, batch_size=36**10)
    y_pred = model.predict(Inputs)
    # print("NN Training Time: === ")
    # tt.toc()
    # elapsedTime_nn = tt.toc()
    # print("NN Elapsed Time: ", elapsedTime_nn)
    # print("NN Loop MSE: ", np.linalg.norm(y_pred-Output)/len(y_pred))
    #
    # Flatten 'F' converts [[1, 2], [3, 4]] into [1, 3, 2, 4]
    Weights_Biases = model.get_weights() # returs a numpy list of weights and biases
    # print("WB: ", Weights_Biases)
    WB = list2flat_WB(Weights_Biases)
    # print("WB Flatten: ", WB)
    # print("weights: ", weights)

    #---> ########### Goal points ########## <---#
    if goal_idx == 1:
        xs = np.array([[2.0], [+1.5], [0.0]]) # reference pose
        # xs = np.array([[+2.0], [+3.0], [0.0]])                               # Reference posture.
    elif goal_idx == 2:
        xs = np.array([[+0.5], [2.0], [0.0]])                               # Reference posture.
    elif goal_idx == 3:
        xs = np.array([[+1.5], [0.0], [0.0]])                               # Reference posture.
    elif goal_idx == 4:
        xs = np.array([[+2.0], [+1.0], [0.0]])                               # Reference posture.
    elif goal_idx == 5:
        xs = np.array([[+0.0], [+0.0], [0.0]])                               # Reference posture.
    elif goal_idx == 6:
        xs = np.array([[+0.0], [+0.0], [0.0]])                               # Reference posture.


    args['p'] = np.concatenate((x0, xs, WB), axis=0)                                # set the values of the parameters vector
    # print("args.p: ", args['p'])

    # initial value of the optimization variables
    init_x = reshape(np.transpose(X0),3*(N+1),1)
    init_u = reshape(np.transpose(u0),2*N,1)
    args['x0']  = np.concatenate((init_x, init_u), axis=0)
    # print("args: ", args['x0'])
    # print("args: ", args)
    # tt.tic()
    sol = solver(x0=args['x0'], p=args['p'], lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'])
    # print("sol: ", sol['x'])
    # print("MPC-CBF Time: === ")
    # tt.toc()
    # elapsedTime_MPC_CBF = tt.toc()
    # print("MPC_CBF Elapsed Time: ", elapsedTime_MPC_CBF)

    solu = sol['x'][3*(N+1):]
    solu_full = np.transpose(solu.full())
    u = np.transpose(reshape(solu_full, 2,N))                                    # get controls only from the solution
    # print("u: ", u)

    solx = sol['x'][0:3*(N+1)]; solx_full = np.transpose(solx.full())
    xx1[0:,0:3,mpciter] = np.transpose(reshape(solx_full, 3,N+1))                               # get solution TRAJECTORY
    # print("xx1: ", xx1[:,0:3,mpciter])

    # Optimal Control >> MPC
    u_cl[mpciter,0:] = u[0:1,0:]
    # print("u_mpc: ", u_cl[mpciter,0:])

    t[mpciter] = t0

    # Apply the control and shift the solution
    t0, u0 = shift(T, t0, u)
    x0 = Xr
    #
    xx[0:,mpciter+1:mpciter+2] = x0
    # print("xx: ", xx)

    solX0 = sol['x'][0:3*(N+1)]; solX0_full = np.transpose(solX0.full())
    X0 = np.transpose(reshape(solX0_full, 3,N+1))                                # get solution TRAJECTORY

    # print("u: ", u)
    # Shift trajectory to initialize the next step
    X0 = np.concatenate((X0[1:,0:n_states+1], X0[N-1:N,0:n_states+1]), axis=0)

    # Apply Control Barrier Function (nominal control: u_cl[mpc_iter,0:])
    # position of obstacle
    v_mpc_cbf = u_cl[mpciter,0]
    w_mpc_cbf = u_cl[mpciter,1]
    #
    # Move Robot
    pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    # print(time.clock(), v_mpc_cbf, w_mpc_cbf, Xr[0], Xr[1], Xr[2])
    print(v_mpc_cbf, w_mpc_cbf, Xr[0], Xr[1], Xr[2])
    # APPLY OPTIMAL CONTROL SIGNAL
    move.linear.x = v_mpc_cbf   # apply NMPC optimal linear velocity
    move.angular.z = w_mpc_cbf  # apply NMPC optimal angular velocity
    # NMPC-CBF
    #
    pub.publish(move)
    # time.sleep(T)

    ne = LA.norm(Xr-xs)

    # Stop Condition
    if ne < 0.1:
        goal_idx = goal_idx + 1
        if goal_idx >= ng:
            pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
            # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            move = Twist()
            move.linear.x = 0; move.angular.z = 0
            pub.publish(move)
            print("Robot has arrived to GOAL point!")
            # rospy.spin()


    # print("mpciter, error: ", mpciter, LA.norm(x0-xs))
    mpciter = mpciter + 1



# Stop Robot
move.linear.x = 0.0                                                     # apply first optimal linear velocity
move.angular.z = 0.0                                                    # apply first optimal angular velocity
pub.publish(move)
print("THE END ...!")
