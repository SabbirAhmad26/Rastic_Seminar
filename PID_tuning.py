import time
from cav_for_PID_tuning import CAV

def main():
    #initialize CAV, PID values, and another parameters
    isMain = True #cav is running on main loop, change to have cav run on merging loop
    CAV1 = CAV("limo799")
    CAV1.generate_map(True, 'n', "o")
    
    eprev_lateral_1= 0
    eint_lateral_1 = 0
    e = 0
    transmissionRate = 30
    dt = 1/transmissionRate # or 0.1
    v_ref_CAV1 = 0.5 # set between 0.5 and 0.8
    within_critical_range = False
    line_changed = True
    slow = 0.3
    fast = 0.15
    within_collision_range = False
    exit_collision_range = True
    just_exit = False

    #depending on if the car starts at main path or merging path, initialize different starting paths, points, and PID values
    current = 0
    next = 1
    CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
    current_collision = 0
    next_collision = 1
    current_line = CAV1.lines[current]
    current_end_pt = CAV1.turning_pts[next]
    if current_collision <= len(CAV1.collision_pts)-2:
        current_collision_pt1 = CAV1.collision_pts[current_collision]
    else:
        current_collision_pt1 = (-1, -1)
    if next_collision <= len(CAV1.collision_pts)-1:
        current_collision_pt2 = CAV1.collision_pts[next_collision]
    else:
        current_collision_pt2 = (-1, -1)
        
    while True:
        #if the cav is near a critical point (which are turning corners), set path to a circle, change starting point and PID values to fit
        if abs(CAV1.position_x  - current_end_pt[0])  < CAV1.ranges[next][0] and \
           abs(CAV1.position_z - current_end_pt[1]) < CAV1.ranges[next][1]:
            if next == len(CAV1.all_pts)-1:
                v_ref_CAV1 = 0
                eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1, eprev_lateral_1,eint_lateral_1,dt)
                CAV1.pub.publish(drive_msg_CAV1)
                print("finished running")
                break
            within_critical_range = True
            line_changed = False
            current_line = CAV1.lines[current]
            e = -(((CAV1.position_x - CAV1.circles[next][0])**2 + (CAV1.position_z - CAV1.circles[next][1])**2)**0.5 - CAV1.circles[next][2])
            v_ref_CAV1 = fast
            print("in activation range")

        #when the cav is on a straight path
        else:
            within_critical_range = False
            current_line = CAV1.lines[current]
            e = - (current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            if just_exit == True:
            	#e = 0
            	just_exit = False
            v_ref_CAV1 = fast
            print("straight path, error:", e)

        #once out of the turning point, follow the next line
        if not line_changed and not within_critical_range:
            current = current+1
            next = next+1
            line_changed = True
            within_critical_range = False
            v_ref_CAV1 = fast
            current_line = CAV1.lines[current]
            current_end_pt = CAV1.turning_pts[next]
            CAV1.kp, CAV1.ki, CAV1.kd = CAV1.PIDs[current]
            eprev_lateral_1= 0
            eint_lateral_1 = 0
            e = (current_line[0]*CAV1.position_x + current_line[1]*CAV1.position_z + current_line[2])/((current_line[0]**2 + current_line[1]**2)**0.5)
            just_exit = True
         
         
         #increament collision points as they are traversed
        if abs(CAV1.position_x  - current_collision_pt1[0])  < CAV1.lane_width/2 and \
            abs(CAV1.position_z - current_collision_pt1[1]) < CAV1.lane_width/2:
            within_collision_range = True
            exit_collision_range = False
        else:
            exit_collision_range = True

        if within_collision_range and exit_collision_range:
            current_collision = current_collision+1
            next_collision = next_collision+1
            exit_collision_range = True
            within_collision_range = False
            if current_collision < len(CAV1.collision_pts)-2:
                current_collision_pt1 = CAV1.collision_pts[current_collision]
            else:
                current_collision_pt1 = (-1, -1)
            if next_collision < len(CAV1.collision_pts)-1:
                current_collision_pt2 = CAV1.collision_pts[next_collision]
            else:
                current_collision_pt2 = (-1, -1)
                
                
        #calculate steering and publisher to the listener node on the limo
        eprev_lateral_1,eint_lateral_1,drive_msg_CAV1 = CAV1.control(e,v_ref_CAV1, eprev_lateral_1,eint_lateral_1,dt)
        CAV1.pub.publish(drive_msg_CAV1)
        dist = calc_distance((CAV1.position_x, CAV1.position_z), (0 ,0))
        #print(dist)

        time.sleep(dt)
        
def calc_distance(pt_1, pt_2):
        distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
        return distance

if __name__ == '__main__':
    main()

    
   
