from sympy import *
from time import time
from mpmath import radians
import tf
from DHmodel import *

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!

    global dh_model
    dh_model.init_variables()

    # DH Parameters

    dh_model.init_dh_parameters()

    # Create invididual transformation matrices

    dh_model.init_transform_matrices()

    # Exctract end-effector position and orientation from request
    
    # px, py, pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px, py, pz = get_position(req.poses[x])

    (roll, pitch, yaw) = get_euler(req.poses[x])


    ROT_EE = dh_model.get_rot_end_effector()
    # More information can be found in KR210 Forward Kinematics section

    Rot_Error = dh_model.get_rot_error(radians(180), radians(-90))

    ROT_EE = ROT_EE * Rot_Error
    ROT_EE = ROT_EE.subs({'r':roll, 'p':pitch, 'y':yaw})

    EE = Matrix([
        [px],
        [py],
        [pz]
        ])

    WC = EE - (0.303) * ROT_EE[:,2]

    # Calculate joint angles using Geometric IK method
    # More information can be found in the Inverse Kinematics with Kuka KR210
    theta1 = atan2(WC[1], WC[0])

    # SSS triangle for theta 2 and theta3
    side_a = 1.501
    side_b = sqrt(pow((sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35),2) + pow((WC[2]-0.75),2))
    side_c = 1.25

    angle_a = acos((side_b*side_b+side_c*side_c-side_a*side_a)/(2*side_b*side_c))
    angle_b = acos((side_a*side_a+side_c*side_c-side_b*side_b)/(2*side_a*side_c))
    angle_c = acos((side_a*side_a+side_b*side_b-side_c*side_c)/(2*side_a*side_b))

    theta2 = pi / 2 - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
    theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m

    R0_3 = dh_model.T0_1[0:3,0:3] * dh_model.T1_2[0:3,0:3] * dh_model.T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={dh_model.q1:theta1, dh_model.q2:theta2, dh_model.q3:theta3})

    R3_6 = R0_3.transpose() * ROT_EE

    # Euler angles from rotation matrix
    # More information can be found in the Euler Angles from a Rotation Matrix section

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    a = sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2])
    theta5 = atan2(a, R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ## Ending at: Populate response for the IK request

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = dh_model.T0_EE.evalf(subs={dh_model.q1:theta1, dh_model.q2:theta2, dh_model.q3:theta3, dh_model.q4:theta4, dh_model.q5:theta5, dh_model.q6:theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0],WC[1],WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":

    ## Change test case number for different scenarios

    global dh_model

    dh_model = DHmodel()

    #test_case_number = 1
    #test_code(test_cases[test_case_number])

    # Try all test cases
    '''
    '''
    for i in range(1, 4):
        print("CASE", i)
        test_code(test_cases[i])

