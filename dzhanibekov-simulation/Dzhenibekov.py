from numpy import *
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from rigidbodyanimation import Point3D, RigidBody

class DzhanibekovEffect():    
    def __init__(self, J1, J2, J3, phi_0, theta_0, psi_0):
        # define instance variables
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.phi_0 = phi_0
        self.theta_0 = theta_0
        self.psi_0 = psi_0
        
        # define moment of inertia matrix
        self.J = array([[J1, 0, 0],
                        [0, J2, 0],
                        [0, 0, J3]])

    def solve_eom(self, t, x):
        phi, theta, psi = x
        
        H = 1
        
        thetadot = H*((1/self.J3)-(1/self.J2))*sin(theta)*sin(psi)*cos(psi)
        phidot =   H*(((sin(psi)**2)/self.J3)+(((cos(psi)**2))/self.J2))
        psidot =   H*((1/self.J1)-((sin(psi)**2)/self.J3)-((cos(psi)**2)/self.J2))*cos(theta)

        xdot = [phidot, thetadot, psidot]
        return xdot


def main():
    # define initial conditions and parameters
    t = linspace(0, 100, 400)
    noise = .1
    J1 = 1
    J2 = 2
    J3 = 3
    phi_0 = 0
    theta_0 = pi/2 + noise
    psi_0 = 0 - noise
    
    # create instance of class
    instance = DzhanibekovEffect(J1, J2, J3, phi_0, theta_0, psi_0)

    # set solving parameters
    tspan = [t[0], t[-1]]
    x0 = [phi_0, theta_0, psi_0]
    
    # solve ode
    sol = solve_ivp(instance.solve_eom, tspan, x0, t_eval=t, rtol=1e-6, atol=1e-9)

    # post processing
    phi = sol.y[0]
    theta = sol.y[1]
    psi = sol.y[2]
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
        rigid_body.euler_rotate(phi[i],theta[i],psi[i])
        rigid_body.update_plot()
        plt.pause(.001)

if __name__ == "__main__":
    main()