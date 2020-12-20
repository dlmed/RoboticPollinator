'''
*************************
Robotic Pollinator: Mechanics, Planning, Computer Vision and Serial Communication
*************************
Authors: Daniel Medina
Email: dlmedina@uc.cl
Date: December 2020
*************************
'''

from pytransform3d.transform_manager import TransformManager
import matplotlib.pyplot as plt
import modern_robotics as mr
import numpy as np
import sympy as sy


class Frames:
    def __init__(self):
        self.alpha = 0.082   # rad
        self.beta = 1.31     # rad
        self.gamma = 1.5708  # rad
        self.delta = 0.10472 # rad

        self.L_pol = 0.05 # 5cm to pollinate
        self.h_cam = 0.0408 # vertical dist. from upper hole center to camera lens
        self.f_cam = 0.0123 # horizontal distance ...

        self.L1 = 0.2265  # m
        self.L2 = 0.265 # m                              # to be measured
        self.L3 = 0.13555  + self.L_pol*np.cos(self.delta) # m
        self.L4 = 0.030557 + self.L_pol*np.sin(self.delta) # m
        self.L5 = 0.05703
        self.L6 = 0.04775


        self.Slist = np.array([[ 0, 0, 1,       0,                    0, 0],    # S_1^0
                               [ 0,-1, 0,       0,                    0, 0],    # S_2^0
                               [ 0,-1, 0, self.L1,                    0, 0],    # S_3^0
                               [-1, 0, 0,       0, -(self.L1 + self.L2), 0]]).T # S_4^0

    def T_01(self, theta1):
        
        return np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                         [np.sin(theta1),  np.cos(theta1), 0, 0],
                         [             0,               0, 1, 0],
                         [             0,               0, 0, 1]])
    
    def T_12(self, theta2):

        return np.array([[-np.sin(theta2), -np.cos(theta2),  0, 0],
                         [              0,               0, -1, 0],
                         [ np.cos(theta2), -np.sin(theta2),  0, 0],
                         [              0,               0,  0, 1]])

    def T_23(self, theta3):

        return np.array([[np.cos(theta3), -np.sin(theta3), 0, self.L1],
                         [np.sin(theta3),  np.cos(theta3), 0,       0],
                         [             0,               0, 1,       0],
                         [             0,               0, 0,       1]])
    
    def T_34(self, theta4):

        return np.array([[ np.cos(theta4), -np.sin(theta4), 0, self.L2],
                         [              0,               0, 1,       0],
                         [-np.sin(theta4), -np.cos(theta4), 0,       0],
                         [              0,               0, 0,       1]])

    def T_45(self):

        return np.array([[np.cos(self.delta), 0, -np.sin(self.delta), self.L3],
                         [                 0, 1,                   0,       0],
                         [np.sin(self.delta), 0,  np.cos(self.delta), self.L4],
                         [                 0, 0,                   0,       1]])

    def T_46(self):

        return np.array([[1, 0, 0, -self.L5 + self.f_cam],
                         [0, 1, 0,                     0],
                         [0, 0, 1,  self.L6 + self.h_cam],
                         [0, 0, 0,                     1]])

    def T_67(self, L7, L8, L9):

        return np.array([[1, 0, 0, L7],
                         [0, 1, 0, L8],
                         [0, 0, 1, L9],
                         [0, 0, 0,  1]])

    def T_mult(self, Ts):
        T = np.matmul(Ts[0], Ts[1])
        if len(Ts) > 2:
            for i in range(2, len(Ts)):
                T = np.matmul(T, Ts[i])
        
        return T

    def FKinEE(self, theta1, theta2, theta3, theta4):

        return self.T_mult([self.T_01(theta1), self.T_12(theta2), \
            self.T_23(theta3), self.T_34(theta4), self.T_45()])
    
    def FKinOBJ(self, theta1, theta2, theta3, theta4, L7, L8, L9):

        return self.T_mult([self.T_01(theta1), self.T_12(theta2), \
            self.T_23(theta3), self.T_34(theta4), self.T_46(), \
            self.T_67(L7, L8, L9)])
    
    def AddTrans(self, theta1, theta2, theta3, theta4, L7, L8, L9):

        self.tm = TransformManager()
        self.tm.add_transform( 'ee',   '4',           self.T_45())
        self.tm.add_transform('cam',   '4',           self.T_46())
        self.tm.add_transform('obj', 'cam', self.T_67(L7, L8, L9))
        self.tm.add_transform(  '1',   '0',     self.T_01(theta1))
        self.tm.add_transform(  '2',   '1',     self.T_12(theta2))
        self.tm.add_transform(  '3',   '2',     self.T_23(theta3))
        self.tm.add_transform(  '4',   '3',     self.T_34(theta4))
    
    def DrawFrames(self, sol=False):

        ax = self.tm.plot_frames_in('0', s=0.1)
        ax.set_xlim((-0.25, 0.75))
        ax.set_ylim((-0.5, 0.5))
        ax.set_zlim((0.0, 1.0))
        if sol:
            ax = self.tm.plot_connections_in('0', ax, \
                whitelist=['0','1','2','3','4','ee','cam',"1'","2'","3'","4'","ee'"])
        else:
            ax = self.tm.plot_connections_in('0', ax, \
                whitelist=['0','1','2','3','4','ee','cam'])
        plt.show()
    
    def IKinEE2(self, theta1, theta2, theta3, theta4, L7, L8, L9, choose, draw):
        pos = self.FKinOBJ(theta1, theta2, theta3, theta4, L7, L8, L9)
        pos = (pos[0][3], pos[1][3], pos[2][3])
        #print(pos)
        thetas0 = [theta1, theta2, theta3, theta4]

        th2, th3, th4 = sy.symbols('th2, th3, th4')
        solutions = []

        for i in np.linspace(-np.pi/2, np.pi/2, 20):
            th1 = i

            eq1 = -self.L1*sy.sin(th2)*sy.cos(th1) + self.L2*(-sy.sin(th2)*sy.cos(th1)*sy.cos(th3) - \
                sy.sin(th3)*sy.cos(th1)*sy.cos(th2)) + self.L3*((-sy.sin(th2)*sy.cos(th1)*sy.cos(th3) - \
                sy.sin(th3)*sy.cos(th1)*sy.cos(th2))*sy.cos(th4) - sy.sin(th1)*sy.sin(th4)) + \
                self.L4*(sy.sin(th2)*sy.sin(th3)*sy.cos(th1) - sy.cos(th1)*sy.cos(th2)*sy.cos(th3)) - pos[0]

            eq2 = -self.L1*sy.sin(th1)*sy.sin(th2) + self.L2*(-sy.sin(th1)*sy.sin(th2)*sy.cos(th3) - \
                sy.sin(th1)*sy.sin(th3)*sy.cos(th2)) + self.L3*((-sy.sin(th1)*sy.sin(th2)*sy.cos(th3) - \
                sy.sin(th1)*sy.sin(th3)*sy.cos(th2))*sy.cos(th4) + sy.sin(th4)*sy.cos(th1)) + \
                self.L4*(sy.sin(th1)*sy.sin(th2)*sy.sin(th3) - sy.sin(th1)*sy.cos(th2)*sy.cos(th3)) - pos[1]

            eq3 = self.L1*sy.cos(th2) + self.L2*(-sy.sin(th2)*sy.sin(th3) + sy.cos(th2)*sy.cos(th3)) + \
                self.L3*(-sy.sin(th2)*sy.sin(th3) + sy.cos(th2)*sy.cos(th3))*sy.cos(th4) + \
                self.L4*(-sy.sin(th2)*sy.cos(th3) - sy.sin(th3)*sy.cos(th2)) - pos[2]
            
            try:
                thetas = sy.nsolve((eq1, eq2, eq3), (th2, th3, th4), (thetas0[1:]))
                the2, the3, the4 = thetas[0], thetas[1], thetas[2]
                solutions.append([float(th1), float(the2), float(the3), float(the4)])
            except:
                pass
        
        if len(solutions) > 0:
            filtered_sols = [sol for sol in solutions if self.filter_range(sol)] # filter ranges
            filtered_sols = [sol for sol in filtered_sols if self.filter_orientation(sol[3], L8)] # filter orientation
            #for sol in filtered_sols:
            #    print(sol)
            #print('\n')

            if choose:
                chsen_sol = self.choose_thetas([theta1, theta2, theta3, theta4], filtered_sols, error=False)
                if draw:
                    self.tm.add_transform("ee'", "4'",             self.T_45())
                    self.tm.add_transform( "1'",  "0", self.T_01(chsen_sol[0]))
                    self.tm.add_transform( "2'", "1'", self.T_12(chsen_sol[1]))
                    self.tm.add_transform( "3'", "2'", self.T_23(chsen_sol[2]))
                    self.tm.add_transform( "4'", "3'", self.T_34(chsen_sol[3]))
                
                return (chsen_sol, True)
            
            else:
                if draw:
                    for i in range(len(filtered_sols)):
                        new_T_ee = self.FKinEE(filtered_sols[i][0], filtered_sols[i][1], filtered_sols[i][2], filtered_sols[i][3])
                        self.tm.add_transform(f"ee'_{i}", "0", new_T_ee)

                return (filtered_sols, True)

        else:
            return (solutions, False)
        
    
    def filter_range(self, th):
        th1_range = (-np.pi/2, np.pi/2)
        th2_range = (-np.pi/2, np.pi/2)
        th3_range = (-1.815, 1.815)
        th4_range = (-np.pi/2, np.pi/2)
        is_in =  (th1_range[0]<th[0]<th1_range[1]) and (th2_range[0]<th[1]<th2_range[1]) and \
                (th3_range[0]<th[2]<th3_range[1]) and (th4_range[0]<th[3]<th4_range[1])

        return is_in

    def filter_orientation(self, th4, L8):
        if L8 >= 0:
            return th4 > 0
        if L8 < 0:
            return th4 < 0
    
    def choose_thetas(self, current_th, sols, error=False):
        if error:
            errors = []
            for sol in sols:
                err1 = abs((current_th[0] - sol[0])/sol[0]) # not divided by curr_th bc it can be 0
                err2 = abs((current_th[1] - sol[1])/sol[1])
                err3 = abs((current_th[2] - sol[2])/sol[2])
                #err4 = abs((current_th[3] - sol[3])/sol[3])
                #err = err1 + err2 + err3 + err4
                err = err1 + err2 + err3
                errors.append(err)
            min_err_index = errors.index(min(errors))

            return sols[min_err_index]
        else:
            th4s = [abs(sol[3]) for sol in sols]
            max_th4_index = th4s.index(max(th4s))

            return sols[max_th4_index]
    
    


if __name__ == '__main__':
    frames = Frames()
    thetas = [0, np.pi/6, -np.pi/2, 0]
    ls = [0.25, -0.2, 0.1]
    frames.AddTrans(*thetas, *ls)

    sols, founded = frames.IKinEE2(*thetas, *ls, choose=True, draw=True)
    print(sols)
    if founded:
        frames.DrawFrames(sol=True)
    
    
