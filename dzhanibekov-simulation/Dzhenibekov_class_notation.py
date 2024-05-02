from numpy import *
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from rigidbodyanimation import Point3D, RigidBody

class DzhanibekovEffect():    
    def __init__(self, J1, J2, J3):
        # define instance variables
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        
        # define moment of inertia matrix
        self.J = array([[J1, 0, 0],
                        [0, J2, 0],
                        [0, 0, J3]])
        
    def euler2rot(self, phi, theta, psi):
        R1phi = array([[1,        0,         0],
                       [0, cos(phi), -sin(phi)],
                       [0, sin(phi),  cos(phi)]])
        
        R3theta = array([[cos(theta), -sin(theta), 0],
                         [sin(theta),  cos(theta), 0],
                         [         0,           0, 1]])
        
        R1psi = array([[1,        0,         0],
                       [0, cos(psi), -sin(psi)],
                       [0, sin(psi),  cos(psi)]])
        
        return R1phi @ R3theta @ R1psi

    def solve_eom(self, t, x):
        w1, w2, w3, r11,r12,r13, r21,r22,r23, r31,r32,r33 = x
        
        w = array([[w1],
                   [w2],
                   [w3]])
        
        Omega = array([[  0, -w3,  w2],
                       [ w3,   0, -w1],
                       [-w2,  w1,   0]])
        
        wdot = linalg.inv(self.J) @ (-Omega @ self.J @ w)
        
        R = array([[r11, r12, r13],
                   [r21, r22, r23],
                   [r31, r32, r33]])
        
        Rdot = R @ Omega
        
        xdot = concatenate((wdot.flatten(), Rdot.flatten()))
        
        return xdot


def main():
    # define initial conditions and parameters
    t = linspace(0, 100, 400)
    noise = .1
    J1 = 1
    J2 = 5
    J3 = 10
    phi_0 = 0
    theta_0 = pi/2 + noise
    psi_0 = 0 - noise
    w1_0 = .01
    w2_0 = 1
    w3_0 = .01
    
    # create instance of class
    instance = DzhanibekovEffect(J1, J2, J3)
    R0 = instance.euler2rot(phi_0,theta_0,psi_0)

    # set solving parameters
    tspan = [t[0], t[-1]]
    x0 = [w1_0, w2_0, w3_0] + ndarray.flatten(R0).tolist()
    
    # solve ode
    sol = solve_ivp(instance.solve_eom, tspan, x0, t_eval=t, rtol=1e-6, atol=1e-9)

    # post processing
    r11 = sol.y[3]
    r12 = sol.y[4]
    r13 = sol.y[5]
    r21 = sol.y[6]
    r22 = sol.y[7]
    r23 = sol.y[8]
    r31 = sol.y[9]
    r32 = sol.y[10]
    r33 = sol.y[11]
    
    t = sol.t

    # animate
    # Define initial coordinates
    initial_x_A, initial_y_A, initial_z_A = 1, 0, 0
    initial_x_B, initial_y_B, initial_z_B = 0, 1, 0
    initial_x_C, initial_y_C, initial_z_C = 0, 0, 1
    initial_x_D, initial_y_D, initial_z_D = -1, 0, 0
    initial_x_E, initial_y_E, initial_z_E = 0, -1, 0
    initial_x_F, initial_y_F, initial_z_F = 0, 0, -1

    # Create points
    point_A = Point3D("Point A", initial_x_A, initial_y_A, initial_z_A)
    point_B = Point3D("Point B", initial_x_B, initial_y_B, initial_z_B)
    point_C = Point3D("Point C", initial_x_C, initial_y_C, initial_z_C)
    point_D = Point3D("Point D", initial_x_D, initial_y_D, initial_z_D)
    point_E = Point3D("Point E", initial_x_E, initial_y_E, initial_z_E)
    point_F = Point3D("Point F", initial_x_F, initial_y_F, initial_z_F)

    # Create rigid body
    rigid_body = RigidBody("My Rigid Body")

    # Add points to rigid body
    rigid_body.add_point(point_A)
    rigid_body.add_point(point_B)
    rigid_body.add_point(point_C)
    rigid_body.add_point(point_D)
    rigid_body.add_point(point_E)
    rigid_body.add_point(point_F)

    # Initialize the plot
    rigid_body.init_plot()

    # Animate the plot
    for i in range(len(t)):
        R = array([[r11[i], r12[i], r13[i]],
                   [r21[i], r22[i], r23[i]],
                   [r31[i], r32[i], r33[i]]])
        rigid_body.rotmat_rotate(R)
        rigid_body.update_plot()
        plt.pause(.001)

if __name__ == "__main__":
    main()