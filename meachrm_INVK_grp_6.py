import numpy as np
from sympy  import symbols, cos, sin, atan2, pi, Matrix, lambdify 
from scipy.optimize import least_squares

q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

DH=[[0,   0,        114,  q1],
    [0,   -pi/2,    0,    q2-(pi/2)],
    [100, 0,        0,    q3],
    [10,  -pi/2,    96,   q4],
    [0,   pi/2,     0,    q5],
    [0,   -pi/2,    56.5, q6]]

def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([[cos(theta),            -sin(theta),            0,             a         ],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha),  cos(alpha)*d ],
                [0,                     0,                      0,            1          ]])
    return M

# Calculate overall transformation matrix
T = Matrix(get_transformation_matrix(DH[0][0],DH[0][1], DH[0][2], DH[0][3] ))
for i in range(1, len(DH)):
    T = T * get_transformation_matrix(DH[i][0], DH[i][1], DH[i][2], DH[i][3])\

# Define symbolic joint variables for differentiation
q_sym = symbols('q1:7')


def symbolic_forward_kinematics(q_values):
    T_symbolic = T.subs({q_sym: q_values})
    return T_symbolic

#You can use this line of code as is
forward_kinematics_func = lambdify(q_sym,  symbolic_forward_kinematics(q_sym), 'numpy')

def position_error(q_position, x_target, y_target, z_target, link_lengths):
    # Calculate forward kinematics for the first three links
    T_values = forward_kinematics_func(q_position[0], q_position[1], q_position[2], 0, 0, 0)
    # Extract the end-effector position from the transformation matrix
    X, Y, Z = T_values[0,3], T_values[1,3], T_values[2,3]
    #return an array of differences between calculated and target positions
    return[X-x_target,Y-y_target, Z-z_target]

def orientation_error(q_orientation, rx_d, ry_d, rz_d):
    # Calculate forward kinematics for the last three links
    T_values = forward_kinematics_func(0, 0, 0, q_orientation[0], q_orientation[1], q_orientation[2])
    # Complete the trig equations
    #  Extract the end-effector orientation from the transformation matrix
    R32 = T_values[2,1]
    R33 = T_values[2,2]
    R31 = T_values[2,0]
    R21 = T_values[1,0]
    R11 = T_values[0,0]
    roll, pitch, yaw = np.arctan2(R32,R33), np.arctan2(-R31,np.sqrt(R32**2+R33**2)), np.arctan2(R21,R11)
    #return an array of differences between calculated and target orientations
    return [roll-rx_d,pitch-ry_d,yaw-rz_d]

def inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init, link_lengths, max_iterations=100, tolerance=1e-6):
    # Perform numerical inverse kinematics for position
    position_args = (x_target,y_target,z_target,link_lengths)
    q_position_solution = least_squares(position_error, q_init[:3], args=position_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x
    # Perform numerical inverse kinematics for orientation
    orientation_args = (rx_d,ry_d,rz_d)
    q_orientation_solution = least_squares(orientation_error, q_init[3:], args=orientation_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x
    #discussion question: We have discussed the concept of Jacobian Matrix in lecture and this document. Where do you think the Jacobian matrix is implemented in this code?
    # Combine the position and orientation components to get the final joint angles
    joint_angles = np.concatenate((q_position_solution, q_orientation_solution))
    return joint_angles

# Example 
x_target = 165.4
y_target = -126.8
z_target = 52.3
rx_d = np.radians(-177.68)  # Roll angle (in radians)
ry_d = np.radians(23.97)  # Pitch angle (in radians)
rz_d = np.radians(143.06)  # Yaw angle (in radians)
q_init = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
link_lengths = [114,100,10,96,56.5]
joint_angles = inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init, link_lengths)

# Convert the joint angles from radians to degrees
# use np.degrees()
#output the joint angles. You may save the output to a csv file

print("Joint Angles (Degrees):")
print("q1 =",  np.degrees(joint_angles[0]))
print("q2 =",  np.degrees(joint_angles[1]))
print("q3 =",  np.degrees(joint_angles[2]))
print("q4 =",  np.degrees(joint_angles[3]))
print("q5 =",  np.degrees(joint_angles[4]))
print("q6 =",  np.degrees(joint_angles[5]))


# need to add validity check #




