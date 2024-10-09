#!/usr/bin/env python3
# coding=UTF-8
import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive

class CAV():
    def __init__(self, node_name):
        self.node_name = node_name
        self.position_z = 0
        self.position_x = 0
        self.position_yaw = 0
        self.velocity = 0
        self.acceleration = 0
        self.Receivedata = 0
        self.position_ip_z = 0
        self.position_ip_x = 0
        self.ip_velocity = 0
        self.ip_acceleration = 0
        self.kp=0
        self.ki=0
        self.kd = 0
        #construct node, subscribe and publish to corrsponding rostopics
        rospy.init_node("listen_pos", anonymous=True)
        self.sub = rospy.Subscriber('/vrpn_client_node/'+self.node_name+'/pose', PoseStamped, self.callback)
        self.pub = rospy.Publisher('vel_steer_'+self.node_name,AckermannDrive,queue_size=10) #topic name = CAV_Data
        rospy.Rate(10)

    def callback(self, msg):
        self.position_z = msg.pose.position.z*1000
        self.position_x = msg.pose.position.x*1000
        self.position_yaw = 0
        self.Receivedata=1

    def generate_map(self, enter=0, exit=0):
        if (exit == 0):
            self.loop = True
        else:
            self.loop = False

        self.lane_width = 450
        self.pt_a = (2940, -1890)
        self.pt_b = (3008, 2600)
        self.pt_c = (1914, -1996)
        self.pt_d = (1826, -650)
        self.pt_e = (1909, 2507)
        self.pt_f = (706, 2662)
        self.pt_g = (183, 280.17)
        self.pt_h = (451, 640)
        self.pt_i = (1600, -1837)
        self.pt_j = (-2018.5, -1957)
        self.pt_j1 = (-2036, -216)
        self.pt_k = (-2027, 270.1)
        self.pt_l = (-2036, 634 )
        self.pt_m = (-2035, 2563)
        self.pt_n = (-2453, -1917)
        self.pt_o = (-2500, 276.1)
        self.pt_p = (-2573, 669)
        self.pt_q = (-2399, 2527)
        self.pt_r = (-4712, -1985)
        self.pt_s = (-4690, 277.6)
        self.pt_t = (-4631, 630)
        self.pt_u = (-4630, 2507)








        #equations for each line, in the A B C form, each variable is a tuple (A, B, C)
        self.path_A = self.generate_line(self.pt_a, self.pt_r)
        self.path_B = self.generate_line(self.pt_a, self.pt_b)
        self.path_C = self.generate_line(self.pt_b, self.pt_u)
        self.path_D = self.generate_line(self.pt_c, self.pt_e)
        self.path_E = self.generate_line(self.pt_d, self.pt_f)
        self.path_F = self.generate_line(self.pt_i, self.pt_h)
        self.path_G = self.generate_line(self.pt_j, self.pt_m)
        self.path_H = self.generate_line(self.pt_n, self.pt_q)
        self.path_I = self.generate_line(self.pt_g, self.pt_s)
        self.path_J = self.generate_line(self.pt_h, self.pt_t)
        self.path_K = self.generate_line(self.pt_r, self.pt_u)


        #data points that characterize each circle for the corners - center x, center y, radius. Each variable is a tuple (A, B, C)
        self.circle_a = (self.pt_a[0]- self.lane_width*1.2, self.pt_a[1] + self.lane_width*2.1, self.lane_width/2)
        self.circle_b = (self.pt_b[0] - self.lane_width, self.pt_b[1] - self.lane_width, self.lane_width/1.8)
        self.circle_c = (self.pt_c[0] + self.lane_width, self.pt_c[1] + self.lane_width, self.lane_width/2)
        self.circle_d = (self.pt_d[0] + self.lane_width, self.pt_d[1] - self.lane_width, self.lane_width) #in practice this is not use
        self.circle_e = (self.pt_s[0] + self.lane_width*2.1, self.pt_s[1] - self.lane_width*1.2, self.lane_width*2)
        self.circle_f = (self.pt_f[0] + self.lane_width, self.pt_f[1] - self.lane_width, self.lane_width/2.2)
        self.circle_g = (self.pt_d[0] - self.lane_width, self.pt_d[1] - self.lane_width, self.lane_width)
        self.circle_h = (self.pt_h[0] - self.lane_width, self.pt_h[1] - self.lane_width, self.lane_width/1.8)
        self.circle_i = (self.pt_i[0] + self.lane_width, self.pt_i[1] + self.lane_width, self.lane_width/1.2)
        self.circle_j = (self.pt_j[0] + self.lane_width, self.pt_j[1] + self.lane_width, self.lane_width/2)
        self.circle_j1 = (self.pt_j1[0] + self.lane_width, self.pt_j1[1] + self.lane_width, self.lane_width/2)
        self.circle_k = (self.pt_k[0] + self.lane_width*2.1, self.pt_k[1] - self.lane_width*1.2, self.lane_width*2)
        self.circle_l = (self.pt_l[0] + self.lane_width, self.pt_l[1] + self.lane_width, self.lane_width/2)
        self.circle_m = (self.pt_s[0] + self.lane_width*2.1, self.pt_s[1] - self.lane_width*1.2, self.lane_width*2)
        self.circle_n = (self.pt_n[0] - self.lane_width*1.2, self.pt_n[1] + self.lane_width*2.1, self.lane_width/2)
        self.circle_o =  (self.pt_o[0] - self.lane_width, self.pt_o[1] - self.lane_width, self.lane_width/1.8)
        self.circle_p = (self.pt_p[0] - self.lane_width*2.8, self.pt_p[1] + self.lane_width*2.1, self.lane_width*1.1) #smaller multiplier to x increases x & decrease the multiplier to increase z
        self.circle_q = (self.pt_q[0] - self.lane_width, self.pt_q[1] - self.lane_width, self.lane_width)
        self.circle_r = (self.pt_r[0] + self.lane_width, self.pt_r[1] + self.lane_width, self.lane_width/2)
        self.circle_s = (self.pt_s[0] + self.lane_width*2.1, self.pt_s[1] - self.lane_width*1.2, self.lane_width*2)
        self.circle_t = (self.pt_t[0] + self.lane_width, self.pt_t[1] + self.lane_width, self.lane_width/2)
        self.circle_u = (self.pt_u[0] + self.lane_width*2.1, self.pt_u[1] - self.lane_width*1.2, self.lane_width*2)

        #the ranges near each corner that activates the circle path for the limo to follow
        self.act_range_a = (self.lane_width * 1.3, self.lane_width /1.7)
        self.act_range_b = (self.lane_width /1.3, self.lane_width * 1.17)
        self.act_range_c = (self.lane_width / 1.5, self.lane_width * 1.5)
        self.act_range_d = (self.lane_width*1 , self.lane_width /2)
        self.act_range_e = (self.lane_width *1.1, self.lane_width/1.7 )
        self.act_range_f = (self.lane_width * 1.2, self.lane_width/1.4)
        self.act_range_g = (self.lane_width * 1, self.lane_width * 1)
        self.act_range_h = (self.lane_width /2, self.lane_width * 1.64)
        self.act_range_i = (self.lane_width /1.5, self.lane_width * 0.6)
        self.act_range_j = (self.lane_width /1.38, self.lane_width * 1.38)
        self.act_range_j1 = (self.lane_width /1.38, self.lane_width * 1.38)

        self.act_range_k = (self.lane_width *1.3, self.lane_width/1.7 )
        self.act_range_l = (self.lane_width / 1.37, self.lane_width * 1.37)
        self.act_range_m = (self.lane_width * 1.25, self.lane_width/1.7 )
        self.act_range_n =  (self.lane_width * 1.3, self.lane_width /1.7)
        self.act_range_o =(self.lane_width / 1.32, self.lane_width * 1.2)
        self.act_range_p = (self.lane_width * 0.85, self.lane_width * 0.85)
        self.act_range_q = (self.lane_width * 1.3, self.lane_width * 1.3)
        self.act_range_r = (self.lane_width /1.5, self.lane_width * 1.37)
        self.act_range_s = (self.lane_width *1.3, self.lane_width/1.7 )
        self.act_range_t = (self.lane_width /1.5, self.lane_width * 1.5)
        self.act_range_u =  (self.lane_width *1.3, self.lane_width/1.7 )


        #values of each line, each element is a tuple (kp, ki, kd)
        self.path_A_PID = (-0.00055, -0.000095, -0.001)
        self.path_B_PID = (0.00055, 0.00003, 0.0009)
        self.path_C_PID = (0.0008, 0.00008, 0.0012)
        self.path_D_PID = (-0.0008, -0.00007, -0.0008)
        self.path_D2_PID = (-0.0005, -0.00042, -0.001)
        self.path_E_PID =(-0.0004, -0.00033, -0.0016)
        self.path_F_PID = (-0.0004, -0.00033, -0.0016)
        self.path_G_PID = (-0.0008, -0.00007, -0.0008)
        self.path_G1_PID = (-0.0008, -0.00007, -0.0008)
        self.path_H_PID = (0.00068, 0.00007, 0.0013)
        self.path_I_PID = (0.0008, 0.00008, 0.0008)
        self.path_J_PID = (-0.00035, -0.000075, -0.0012)
        self.path_K_PID = (-0.0008, -0.00004, -0.001)

        #PID values of each circle, each element is a tuple (kp, ki, kd)
        self.circle_a_PID = (-0.45, -0.00045, -0.037)
        self.circle_b_PID = (-0.45, -0.00045, -0.039)
        self.circle_c_PID = (-0.56, -0.00045, -0.037)
        self.circle_d_PID = (-0.0005, -0.00045, -0.03)
        self.circle_e_PID = (-0.003, -0.000045, -0.0017)
        self.circle_f_PID = (-0.50, -0.00045, -0.037)
        self.circle_g_PID = (-0.25, -0.000005, -0.07)
        self.circle_h_PID =(0.007, 0.0000012, 0.8)
        self.circle_i_PID = (-0.0026, -0.000045, -0.0037)
        self.circle_j_PID = (-0.6, -0.000005, -0.037)
        self.circle_j1_PID = (-0.6, -0.000005, -0.037)
        self.circle_k_PID = (-0.25, -0.000005, -0.07)
        self.circle_l_PID =(-0.6, -0.00045, -0.037)
        self.circle_m_PID = (-0.6, -0.00045, -0.037)
        self.circle_n_PID =(-0.45, -0.00045, -0.037)
        self.circle_o_PID = (-0.45, -0.00045, -0.67)
        self.circle_p_PID =  (-0.25, -0.000005, -0.7)
        self.circle_q_PID = (-0.25, -0.000005, -0.7)
        self.circle_r_PID = (-0.6, -0.00045, -0.037)
        self.circle_s_PID = (-0.6, -0.00045, -0.037)
        self.circle_t_PID = (-0.6, -0.00045, -0.037)
        self.circle_u_PID = (-0.6, -0.00045, -0.37)


        if enter == 'e' and exit == 0 : #if the limo runs along the main path
            #array to store all points at which the limo needs to turn, in order of traversal
            self.turning_pts = [self.pt_e, self.pt_c, self.pt_a, self.pt_b]
            #array to store all lines, in order of traversal
            self.lines = [self.path_D, self.path_A, self.path_B, self.path_C]
            #the activation range of the corners, in order of traversal
            self.ranges = [self.act_range_e, self.act_range_c, self.act_range_a, self.act_range_b]
            #array to store the circles for the corners, in order of traversal
            self.circles = [self.circle_e, self.circle_c, self.circle_a, self.circle_b]
            #array to store PID values of each line, in order of traversal, each element is a tuple (kp, ki, kd)
            self.PIDs = [self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
            #array to store PID values of each circle, in order of traversal, each element is a tuple (kp, ki, kd)
            self.curve_PIDs = [self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID]

        elif enter == 'f' and exit == 0: #if the limo runs along the merging path
            self.turning_pts = [self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b]
            self.lines = [self.path_E, self.path_D, self.path_A, self.path_B, self.path_C]
            self.ranges = [self.act_range_f, self.act_range_d, self.act_range_c, self.act_range_a, self.act_range_b]
            self.circles = [self.circle_f, self.circle_d, self.circle_c, self.circle_a, self.circle_b]
            self.PIDs = [self.path_E_PID, self.path_D2_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID]
            self.curve_PIDs = [self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID]

        elif enter == 'n' and exit == 'o':  # N to U (full route)
            self.turning_pts = [self.pt_n, self.pt_o, self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_e, self.pt_c, self.pt_a, self.pt_b, self.pt_u, self.pt_r, self.pt_n, self.pt_o]
            self.lines = [self.path_H, self.path_I, self.path_K, self.path_A, self.path_B, self.path_C, self.path_D, self.path_A, self.path_B, self.path_C, self.path_K, self.path_A, self.path_H]
            self.ranges = [self.act_range_n, self.act_range_o, self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_e, self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r, self.act_range_n, self.act_range_o]
            self.circles = [self.circle_n_PID, self.circle_o_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID, self.circle_o_PID]
            self.PIDs = [self.path_H_PID, self.path_I_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID, self.path_H_PID]
            self.curve_PIDs = [self.circle_n_PID, self.circle_o_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID, self.circle_n_PID, self.circle_o_PID]

        elif enter == 'n' and exit == 0:  # N to N (full route)
            self.turning_pts = [self.pt_n, self.pt_o, self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_e, self.pt_c, self.pt_a, self.pt_b, self.pt_u, self.pt_r]
            self.lines = [self.path_H, self.path_I, self.path_K, self.path_A, self.path_B, self.path_C, self.path_D, self.path_A, self.path_B, self.path_C, self.path_K, self.path_A]
            self.ranges = [self.act_range_n, self.act_range_o, self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_e, self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_u, self.act_range_r]
            self.circles = [self.circle_n_PID, self.circle_o_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID]
            self.PIDs = [self.path_H_PID, self.path_I_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_A_PID]
            self.curve_PIDs = [self.circle_n_PID, self.circle_o_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_e_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_r_PID]


        elif enter == 'g' and exit == 'u':  # N to U(full route)
            self.turning_pts = [self.pt_g, self.pt_s, self.pt_r, self.pt_a, self.pt_b, self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b, self.pt_m, self.pt_l]
            self.lines = [self.path_I, self.path_K, self.path_A, self.path_B, self.path_C, self.path_E,self.path_D, self.path_A, self.path_B, self.path_C, self.path_G]
            self.ranges = [self.act_range_g, self.act_range_s, self.act_range_r, self.act_range_a, self.act_range_b, self.act_range_f, self.act_range_d,self.act_range_c, self.act_range_a, self.act_range_b,self.act_range_m, self.act_range_l ]
            self.circles = [self.circle_g_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID,self.circle_l_PID]
            self.PIDs = [self.path_I_PID, self.path_K_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_E_PID, self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_G_PID]
            self.curve_PIDs = [self.circle_g_PID, self.circle_s_PID, self.circle_r_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID,self.circle_d_PID,  self.circle_c_PID, self.circle_a_PID,self.circle_b_PID, self.circle_m_PID, self.circle_l_PID]

        elif enter == 't' and exit == 'u':  # T to U (full route)
            self.turning_pts = [self.pt_t, self.pt_h, self.pt_i, self.pt_a, self.pt_b, self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b, self.pt_u,  self.pt_t, self.pt_p]
            self.lines = [self.path_J, self.path_F, self.path_A, self.path_B, self.path_C, self.path_E, self.path_D, self.path_A, self.path_B, self.path_C, self.path_K, self.path_J]
            self.ranges = [self.act_range_t, self.act_range_h, self.act_range_i, self.act_range_a, self.act_range_b, self.act_range_f, self.act_range_d, self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_u,self.act_range_t , self.act_range_p]
            self.circles = [self.circle_t, self.circle_h, self.circle_i, self.circle_a, self.circle_b, self.circle_f, self.circle_d, self.circle_c, self.circle_a, self.circle_b, self.circle_u, self.circle_t, self.circle_p]
            self.PIDs = [self.path_J_PID, self.path_F_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_E_PID, self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_K_PID, self.path_J_PID]
            self.curve_PIDs = [self.circle_t_PID, self.circle_h_PID, self.circle_i_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_u_PID, self.circle_t_PID, self.circle_p_PID]

        elif enter == 'm' and exit == 'u':  # M to U (full route)
            self.turning_pts = [self.pt_m, self.pt_j, self.pt_a, self.pt_b, self.pt_f, self.pt_d, self.pt_c, self.pt_a, self.pt_b, self.pt_m,self.pt_l]
            self.lines = [self.path_G, self.path_A, self.path_B, self.path_C, self.path_E, self.path_D, self.path_A, self.path_B, self.path_C, self.path_G]
            self.ranges = [self.act_range_m, self.act_range_j, self.act_range_a, self.act_range_b, self.act_range_f, self.act_range_d, self.act_range_c, self.act_range_a, self.act_range_b, self.act_range_m, self.act_range_l ]
            self.circles = [self.circle_m_PID, self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID, self.circle_l_PID]
            self.PIDs = [self.path_G_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_E_PID, self.path_D_PID, self.path_A_PID, self.path_B_PID, self.path_C_PID, self.path_G_PID]
            self.curve_PIDs = [self.circle_m_PID, self.circle_j_PID, self.circle_a_PID, self.circle_b_PID, self.circle_f_PID, self.circle_d_PID, self.circle_c_PID, self.circle_a_PID, self.circle_b_PID, self.circle_m_PID, self.circle_l_PID]


    #helper functions for generate_map()
    def generate_line(self, pt_1, pt_2):
        A = -(pt_2[1] - pt_1[1])
        B = -(pt_1[0] - pt_2[0])
        C = -(pt_1[1] * (pt_2[0] - pt_1[0]) - (pt_2[1] - pt_1[1]) * pt_1[0])
        return A, B, C

    def calc_distance(self, pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance

    def calc_dist_array(self, points):
        dist = []
        for i in range(len(points)-1):
            dist.append(self.calc_distance(points[i], points[i+1]))
        return dist

    #helper function for generate_map()
    def control(self,e,v_ref, eprev_lateral,eint_lateral,dt):
        if (eprev_lateral*e<=0):
            eint_lateral = 0
        kp = self.kp
        ki = self.ki
        kd = self.kd

        [ref_steer,u_k ,u_i ,u_d, eprev_lateral, eint_lateral] = self.PIDController(e, eprev_lateral, eint_lateral, dt, kp, ki, kd)

        drive_msg = AckermannDrive()
        drive_msg.speed = v_ref
        drive_msg.steering_angle = self.steeringAngleToSteeringCommand(ref_steer)
        return eprev_lateral,eint_lateral,drive_msg

    def PIDController(self, e, prev_e, prev_int, delta_t, Kp, Ki, Kd): #add theta_ref as input
        if e <= 1 and e>=-1:
            e_int = 0
        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,0.3),-0.3)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # PID controller for omega
        u_k = Kp*e
        u_i = Ki*e_int
        u_d = Kd*e_der
        u = Kp*e + Ki*e_int + Kd*e_der

        return u, u_k, u_i, u_d, e, e_int
    def steeringAngleToSteeringCommand(self,refAngle):
        x = refAngle
        y = 0.7*x
        return y
