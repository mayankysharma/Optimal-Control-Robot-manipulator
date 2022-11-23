import sys
from typing import Union
import math
import os
import logging

from sympy import symbols, Symbol, Matrix, Pow, cos, sin
import sympy
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


logging.basicConfig(level=logging.INFO)

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
from  roboticstoolbox import DHRobot, RevoluteDH

class Scara2DOF():

    def __init__(self):
        ## Assigning DH Parameters for scara Robot
        self.robot = DHRobot([
                    RevoluteDH(a=8),
                    RevoluteDH(a=6),], name="Scara2DOF")
                
        logging.info("\n######\nDH Parameter for Scara 2DOF Robot :")
        logging.info(f"{self.robot}")

    def plot(self,qs, dt, gif_filename):
        axes = np.array([-14,14,-14,14,-14,14])
        self.robot.plot(qs,dt=dt, backend='pyplot', eeframe=True,jointaxes=True,limits=axes, movie=gif_filename)


class Manipulator:

    # state variable
    q1, q2, q1_dot, q2_dot = symbols("q1 q2 q1_dot q2_dot")
    state = Matrix([[q1],[q2],[q1_dot],[q2_dot]])

    # robot variables
    m1, m2 = symbols("m1 m2") # oz
    J1, J2 = symbols("J1 J2") # oz-in/rad/sec^2
    r1, r2 = symbols("r1 r2") # in
    l1, l2 = symbols("l1 l2") # in
    b1, b2 = symbols("b1 b2") # oz-in/rad/sec

    variables = {m1:13.86,m2:3.33,J1: 62.39,J2:16.7, r1:6.12, r2:3.22, l1:8, l2:6, b1:0.2,b2:0.5}
    
    # load 
    e = Symbol("e") # e belongs to [0,1]
    ml = 10*e  # oz
    Jl = 60*Pow(e,2) # oz-in/rad/sec^2

    # inertia Matrix
    M11 = J1+m1*Pow(r1,2)+J2+m2*(Pow(l1,2)+Pow(r2,2)+2*l1*r2*cos(q2))+Jl+ml*(Pow(l1,2)+Pow(l2,2)+2*l1*l2*cos(q2))
    M12 = J2 + m2*(Pow(r2,2)+l1*r2*cos(q2))+Jl+ml*Pow(l2,2)
    M21 = M12
    M22 = J2+m2*Pow(r2,2)+Jl+ml*Pow(l2,2)
    M = Matrix([[M11,M12],[M21,M22]])
    # M = Matrix([M11,M12])
    M0 = M.subs({**variables,**{e:1}})

    # N (friction+centripital force)
    n1 = (m2*l1*r2+ml*l1*l2)*(Pow(q2_dot,2)+2*q1_dot*q2_dot)*sin(q2)+b1*q1_dot
    n2 = (m2*l1*r2+ml*l1*l2)*Pow(q1_dot,2)*sin(q2)+b2*q2_dot
    N = Matrix([[n1],[n2]])
    # N
    N0 = N.subs({**variables,**{e:1}})

    def __init__(self,plot=True, path2Store : Union[str,None]=None,getAnimation : bool = True):
        logging.info("Initializing the Manipulator")
        self.get_dynamics()
        self.get_optimal_control()
        self.path = path2Store
        self.plot = plot
        self.getAnimation = getAnimation
        self.scara = Scara2DOF()

    def get_dynamics(self):
        self.A = Matrix([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
        self.B = Matrix([[0,0],[0,0],[1,0],[0,1]])
        
    def get_optimal_control(self):
        # optimal control input
        X1 = Matrix([[self.q1],[self.q2]])
        K = Matrix([[14.803,1.3591],[1.3591,2.8553]])
        X2 = Matrix([[self.q1_dot],[self.q2_dot]])
        self.U0 = -X1-K*X2

    def get_uncertainties(self,M_sub, N_sub, N0, M0):
        # Matched uncertainty in system dynamics, 
        f = M_sub.inv()*(N0-N_sub)

        # uncertainty input matrix, 
        h = M_sub.inv()*M0-sympy.eye(2)

        return f,h


    def state_space_model(self,state,optimal_input,h,f,dt:float):
        state_dot = self.A*state+self.B*(optimal_input+h*optimal_input)+self.B*f
        # sympy.pprint(state_dot)
        q1 = state[0]+state_dot[0]*dt
        q2 = state[1]+state_dot[1]*dt
        q1_dot = state[2]+state_dot[2]*dt
        q2_dot = state[3]+state_dot[3]*dt
        next_state = Matrix([q1,q2,q1_dot,q2_dot]).reshape(4,1)
        return next_state


    def run_controller(self, total_time:int=50, dt:float=0.01, epsilon:float = 0):
        
        load_variables = {self.e:epsilon}
        
        q1s = []
        q2s = []
        q1_dots = []
        q2_dots = []
        u1s = []
        u2s = []
        Qs = []
        Ts = np.arange(0,total_time,dt)
        logging.info(f"Loop over total_time : {total_time}| and with a time step | dt = {dt}")
        X_states = {self.q1:math.radians(60),self.q2:math.radians(-30),self.q1_dot:0,self.q2_dot:0}
        curr_state = self.state.subs(X_states)
        for i in tqdm(Ts):
            X_states = {self.q1:curr_state[0],self.q2:curr_state[1],self.q1_dot:curr_state[2],self.q2_dot:curr_state[3]}
            # print("\ncurrent_states: ",X_states)
            optimal_control_input = self.U0.subs(X_states)
            M_sub = self.M.subs({**self.variables, **load_variables,**X_states})
            N_sub = self.N.subs({**self.variables, **load_variables,**X_states})
            N0_sub = self.N0.subs(X_states)
            M0_sub = self.M0.subs(X_states)

            f,h = self.get_uncertainties(M_sub,N_sub,N0_sub,M0_sub)
            
            u1s.append(optimal_control_input[0])
            u2s.append(optimal_control_input[1])
            q1s.append(X_states[self.q1])
            q2s.append(X_states[self.q2])
            Qs.append([X_states[self.q1],X_states[self.q2]])
            q1_dots.append(X_states[self.q1_dot])
            q2_dots.append(X_states[self.q2_dot])

            next_state = self.state_space_model(curr_state,optimal_control_input,h,f,dt)
            curr_state=next_state
        if self.plot:
            self.plotData(q1s,q2s,q1_dots,q2_dots,u1s,u2s,Ts,epsilon)
            plt.waitforbuttonpress()
            plt.close()
            if self.getAnimation:
                gif_filename = os.path.splitext(self.path)[0]+".gif"
                Qs = np.array(Qs)
                m = Qs.shape[0]
                if m>100:
                    new_m = m//100
                    Qs = Qs[::new_m,:]
                self.scara.plot(Qs,0.1,gif_filename) # for faster graph multiplying dt with 10
                plt.close()
    def plotData(self,q1s:list,q2s:list,q1_dots:list,q2_dots:list,u1s:list,u2s:list,Ts:list,epsilon:float):
        # plotting
        fig,axs = plt.subplots(nrows=3,ncols=2,figsize=(15,10))
        fig.suptitle(r"time response for $\epsilon$={epsilon}".format(epsilon=epsilon),fontsize=18)
        # print(axs)
        ax1 = axs[0,0]
        ax1.set_xlabel("t (sec)")
        ax1.set_ylabel("q1 (rad)")
        ax1.plot(Ts,q1s)

        ax2 = axs[0,1]
        # ax2 =  plt.subplot(322,autoscale_on=True)
        ax2.set_xlabel("t (sec)")
        ax2.set_ylabel("q2 (rad)")
        ax2.plot(Ts,q2s)

        ax3 = axs[1,0]
        # ax3 =  plt.subplot(323)
        ax3.set_xlabel("t (sec)")
        ax3.set_ylabel("q1dot (rad/sec)")
        ax3.plot(Ts,q1_dots)

        ax4 = axs[1,1]
        # ax4 =  plt.subplot(324)
        ax4.set_xlabel("t (sec)")
        ax4.set_ylabel("q2dot (rad/sec)")
        ax4.plot(Ts,q2_dots)

        ax5 = axs[2,0]
        # ax5 =  plt.subplot(325)
        ax5.set_xlabel("t (sec)")
        ax5.set_ylabel("u1")
        ax5.plot(Ts,u1s)

        ax6 = axs[2,1]
        # ax6 =  plt.subplot(326)
        ax6.set_xlabel("t (sec)")
        ax6.set_ylabel("u2")
        ax6.plot(Ts,u2s)

        if self.path:
            plt.savefig(self.path)
        else:
            plt.show()

if __name__ == "__main__":
    epsilon = 0
    if len(sys.argv)>2:
        path = sys.argv[1]
        epsilon = float(sys.argv[2])
        dirs = os.path.dirname(path)
        if not os.path.isdir(dirs):
            logging.warning(f"Given Directory not exists : {dirs}")
            sys.exit()
        if epsilon < 0 or epsilon > 1:
            logging.warning(f"Value of epsilon : {epsilon} | is not in range [0,1]")
            sys.exit()
    elif len(sys.argv)>1:
        path = sys.argv[1]
        dirs = os.path.dirname(path)
        if not os.path.isdir(dirs):
            logging.warning(f"Given Directory not exists : {dirs}")
            sys.exit()
    else:
        path=None
    # Need Animation make it True else False
    animation = True
    manipulator = Manipulator(path2Store=path,getAnimation=animation)
    logging.info("NOTE : Stabilising the manipulator angles to 0 degree, from q1 = 60 deg and q2 = -30 deg")
    logging.info(f"Running Controller for epsilon = {epsilon}")
    manipulator.run_controller(epsilon=epsilon, total_time=50)