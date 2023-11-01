import numpy as np
from sympy  import symbols, cos, sin, atan2, pi, Matrix, lambdify , simplify
from scipy.optimize import least_squares
from extract_values import read_csv

q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

DH=[[0,   0,        114,  q1],
    [0,   -pi/2,    0,    q2-(pi/2)],
    [100, 0,        0,    q3],
    [10,  -pi/2,    96,   q4],
    [0,   pi/2,     0,    q5],
    [0,   -pi/2,    156.5, q6]] # gripper offset by 100 from previous end effector position, which was 56.5
    

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


def get_new_angles():
    coords_list_Fbase = read_csv('C:/Users/Obert/Documents/final_vals.csv', 'c')

    coords_list_Frover = []

    # base frame in world frame
    T_base = get_transformation_matrix(0, 0 , 31, 0)

    # For trover, maybe set a to 150 or offset distance, otherwise try and place direclty over where base was previously

    # world frame in rover frame
    T_rover = get_transformation_matrix(0, 0, -125, 0)

    T_rover_base = T_base * T_rover  
    for coords in coords_list_Fbase:
        point_Fbase = Matrix([coords[0], coords[1], coords[2], 1])
        point_Frover = T_rover_base * point_Fbase
        new_coords = [point_Frover[0,0], point_Frover[1,0], point_Frover[2,0], coords[3], coords[4], coords[5]]
        coords_list_Frover.append(new_coords)

    #print(coords_list_Frover)
    q_init = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    link_lengths = [114,100,10,96,56.5]


    angle_list = []
    for coords in coords_list_Frover:
        coords = np.array(coords, dtype='float')
        joint_angles = inverse_kinematics(coords[0], coords[1], coords[2], np.radians(coords[3]), np.radians(coords[4]), np.radians(coords[5]), q_init, link_lengths)
        joint_angles = np.degrees(joint_angles)
        modified_angles = []

        # change angle so its absolute value is as small as while mainting the same position
        for angle in joint_angles:
            while abs(angle) >= 360:
                if angle > 0:
                    angle -= 360
                elif angle < 0:
                    angle += 360

            if abs(angle) > 180:
                if angle > 0:
                    angle -= 360
                elif angle < 0:
                    angle += 360

            modified_angles.append(angle)

        angle_list.append(modified_angles)
    return angle_list







