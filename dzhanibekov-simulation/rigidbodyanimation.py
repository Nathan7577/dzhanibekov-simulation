from numpy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Point3D:
    def __init__(self, name, x, y, z):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.x0 = x
        self.y0 = y
        self.z0 = z
        

    def set_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_position(self):
        return self.x, self.y, self.z
    
    def get_initial_position(self):
        return self.x0, self.y0, self.z0

class RigidBody:
    def __init__(self, name):
        self.name = name
        self.points = {}
        self.lines = {}
        self.origin = [0,0,0]

    def add_point(self, point):
        self.points[point.name] = point

    def set_point_position(self, point_name, x, y, z):
        if point_name in self.points:
            self.points[point_name].set_position(x, y, z)

    def init_plot(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Dzhenibekov Effect')
        self.ax.set_xlim([-1.5,1.5])
        self.ax.set_ylim([-1.5,1.5])
        self.ax.set_zlim([-1.5,1.5])
        for point in self.points.values():
            x, y, z = point.get_position()
            self.lines[point.name] = self.ax.plot([self.origin[0],x],[self.origin[1],y],[self.origin[2],z])

    def update_plot(self):
        if self.fig is None or self.ax is None:
            print("Error: Plot not initialized.")
            return

        for point in self.points.values():
            x, y, z = point.get_position()
            self.lines[point.name][0].set_xdata([self.origin[0],x])
            self.lines[point.name][0].set_ydata([self.origin[1],y])
            self.lines[point.name][0].set_3d_properties([self.origin[2],z])
            
        plt.pause(0.001)
        
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

    def rot2euler(R):
        theta = arccos(R[0,0])
        phi = arccos(-R[0,1]/sin(theta))
        psi = arccos(R[1,0]/sin(theta))
        return phi, theta, psi
    
    def euler_rotate(self, phi, theta, psi):
        R = self.euler2rot(phi, theta, psi)        
        for point in self.points.values():
            x0, y0, z0 = point.get_initial_position()
            xyz = R @ array([[x0],[y0],[z0]])
            x = xyz[0,0]
            y = xyz[1,0]
            z = xyz[2,0]
            point.set_position(x,y,z)
            
    def rotmat_rotate(self,R):    
        for point in self.points.values():
            x0, y0, z0 = point.get_initial_position()
            xyz = R @ array([[x0],[y0],[z0]])
            x = xyz[0,0]
            y = xyz[1,0]
            z = xyz[2,0]
            point.set_position(x,y,z)