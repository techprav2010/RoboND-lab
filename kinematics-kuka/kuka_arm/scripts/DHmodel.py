
import tf
from sympy import symbols, cos, sin, pi, sqrt
from sympy.matrices import Matrix
#from mpmath import * # for arbitraty floating point precision operations
#import numpy as np

## General functions

def get_position(p):

    px = p.position.x
    py = p.position.y
    pz = p.position.z
    return px, py, pz

def get_euler(p):

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [p.orientation.x, 
         p.orientation.y, 
         p.orientation.z, 
         p.orientation.w
        ])
    return (roll, pitch, yaw)

# Main Class

class DHmodel():

    def __init__(self):

        self.r, self.p, self.y = symbols('r p y')

        self.ROT_x = None
        self.ROT_y = None
        self.ROT_z = None
        self.ROT_EE = None
        self.Rot_Error = None

    def init_variables(self):

        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7 = symbols('q1:8')
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7 = symbols('d1:8') # link offset
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a0:7') # link length
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7') # twist angle        

    def init_dh_parameters(self):

        self.s = {
            self.alpha0:      0,  self.a0:      0, self.d1:  0.75, self.q1:          self.q1, 
            self.alpha1: -pi/2.,  self.a1:   0.35, self.d2:     0, self.q2: -pi/2. + self.q2,
            self.alpha2:      0,  self.a2:   1.25, self.d3:     0, self.q3:          self.q3,
            self.alpha3: -pi/2.,  self.a3: -0.054, self.d4:   1.5, self.q4:          self.q4,
            self.alpha4:  pi/2.,  self.a4:      0, self.d5:     0, self.q5:          self.q5,
            self.alpha5: -pi/2.,  self.a5:      0, self.d6:     0, self.q6:          self.q6,
            self.alpha6:      0,  self.a6:      0, self.d7: 0.303, self.q7:          0
        }

    # Use a function to generate the DH Transformation matrix
    def TF_Matrix(self, alpha, a, d, q):

        TF = Matrix([
            [cos(q), -sin(q), 0, a],
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [0,0,0,1]
            ])
        return TF

    def sandbox(self):

        ### Sandbox method to evaluate code
        ### Create symbols for joint variables
        q1, q2 = symbols('q1:3')

        R_y = Matrix([[ cos(q1),        0, sin(q1)],
              [ 0,              1,       0],
              [-sin(q1),        0, cos(q1)]])
        R_z = Matrix([[ cos(q2), -sin(q2),       0],
              [ sin(q2),  cos(q2),       0],
              [       0,        0,       1]])

    def init_transform_matrices(self):

        self.T0_1  = self.TF_Matrix(self.alpha0, self.a0, self.d1, self.q1).subs(self.s)
        self.T1_2  = self.TF_Matrix(self.alpha1, self.a1, self.d2, self.q2).subs(self.s)
        self.T2_3  = self.TF_Matrix(self.alpha2, self.a2, self.d3, self.q3).subs(self.s)
        self.T3_4  = self.TF_Matrix(self.alpha3, self.a3, self.d4, self.q4).subs(self.s)
        self.T4_5  = self.TF_Matrix(self.alpha4, self.a4, self.d5, self.q5).subs(self.s)
        self.T5_6  = self.TF_Matrix(self.alpha5, self.a5, self.d6, self.q6).subs(self.s)
        self.T6_EE = self.TF_Matrix(self.alpha6, self.a6, self.d7, self.q7).subs(self.s)

        self.T0_EE = self.T0_1 * self.T1_2 * self.T2_3 * self.T3_4 * self.T4_5 * self.T5_6 * self.T6_EE

    def get_rot_x(self):

        if self.ROT_x is None:
	        self.ROT_x = Matrix([
	            [1, 0, 0],
	            [0, cos(self.r), -sin(self.r)],
	            [0, sin(self.r), cos(self.r)]
	            ]) # Roll
        return self.ROT_x

    def get_rot_y(self):

        if self.ROT_y is None:
	        self.ROT_y = Matrix([
	            [cos(self.p), 0, sin(self.p)],
	            [0, 1, 0],
	            [-sin(self.p), 0, cos(self.p)]
	            ]) # Pitch
        return self.ROT_y

    def get_rot_z(self):

        if self.ROT_z is None:
	        self.ROT_z = Matrix([
	            [cos(self.y), -sin(self.y), 0],
	            [sin(self.r), cos(self.y), 0],
	            [0, 0, 1]
	            ]) # Yaw
        return self.ROT_z

    def get_rot_end_effector(self):

        # Find EE rotation matrix
        # Define RPY rotation matrices
        # http://planning.cs.uiuc.edu/node102.html

        if self.ROT_EE is None:
            self.ROT_EE = self.get_rot_z() * self.get_rot_y() * self.get_rot_x()
        return self.ROT_EE

    def get_rot_error(self, y_angle, p_angle):

        self.Rot_Error = self.ROT_z.subs(self.y, y_angle) * self.ROT_y.subs(self.p, p_angle)
        return self.Rot_Error
