#!/usr/bin/python
import roslib
#roslib.load_manifest('cs225b')
import rospy
import math
from geometry_msgs.msg import WrenchStamped
import numpy as np


alpha = 1.222
pub_l = rospy.Publisher("/ft_transformed/lef_arm",WrenchStamped)
##pub_r = rospy.Publisher("/ft_transformed/rig_arm",WrenchStamped)
##pub_l = rospy.Publisher("/ft_transformed/rig_arm",WrenchStamped)

# Initialization of variables for RIGHT ARM
wrench_aligned_r = WrenchStamped()

Fx_r = 0
Fy_r = 0
Fz_r = 0
Tx_r = 0
Ty_r = 0
Tz_r = 0
bias_Fx_r = 0
bias_Fy_r = 0
bias_Fz_r = 0
bias_Tx_r = 0
bias_Ty_r = 0
bias_Tz_r = 0
bias_computed_r = 0
counter_r = 0

# Initialization of variables for LEFT ARM
wrench_aligned_l = WrenchStamped()

Fx_l = 0
Fy_l = 0
Fz_l = 0
Tx_l = 0
Ty_l = 0
Tz_l = 0
# --- Compute bias -----
bias_computed_l = 0
bias_Fx_l = 0
bias_Fy_l = 0
bias_Fz_l = 0
bias_Tx_l = 0
bias_Ty_l = 0
bias_Tz_l = 0
# ----------------------

# --- Known bias -----
##bias_computed_l = 1
##bias_Fx_l = 14.7566
##bias_Fy_l = -5.2744
##bias_Fz_l = 32.3773
##bias_Tx_l = -0.1050
##bias_Ty_l = -1.0453
##bias_Tz_l = -0.3155
# ---------------------


counter_l = 0

def callback_right_arm(msg):
    wrench_aligned_r = msg
    # Compute bias for the first 1000 samples:
    if bias_computed_r == 0:
        get_bias_r(wrench_aligned_r)
        if counter_r == 1000:
            print 'R_ Bias Fx: ', bias_Fx_r
            print 'R_ Bias Fy: ', bias_Fy_r
            print 'R_ Bias Fz: ', bias_Fz_r
            print 'R_ Bias Tx: ', bias_Tx_r
            print 'R_ Bias Ty: ', bias_Ty_r
            print 'R_ Bias Tz: ', bias_Tz_r
    else:
        unbiased_Fx = wrench_aligned_r.wrench.force.x - bias_Fx_r
        unbiased_Fy = wrench_aligned_r.wrench.force.y - bias_Fy_r
        unbiased_Tx = wrench_aligned_r.wrench.torque.x - bias_Tx_r
        unbiased_Ty = wrench_aligned_r.wrench.torque.y - bias_Ty_r
        wrench_aligned_r.wrench.force.z -= bias_Fz_r
        wrench_aligned_r.wrench.torque.z -= bias_Tz_r
        wrench_aligned_r.wrench.force.x = unbiased_Fx*math.cos(alpha) - unbiased_Fy*math.sin(alpha)
        wrench_aligned_r.wrench.force.y = -(unbiased_Fx*math.sin(alpha)) - (unbiased_Fy*math.cos(alpha))
        wrench_aligned_r.wrench.torque.x = unbiased_Tx*math.cos(alpha) - unbiased_Ty*math.sin(alpha)
        wrench_aligned_r.wrench.torque.y = -(unbiased_Tx*math.sin(alpha)) - (unbiased_Ty*math.cos(alpha))
        pub_r.publish(wrench_aligned_r)
    return


def get_bias_r(wrench_aligned):
    global Fx_r, Fy_r, Fz_r, counter_r, bias_computed_r, bias_Fx_r, bias_Fy_r, bias_Fz_r
    global Tx_r, Ty_r, Tz_r, bias_Tx_r, bias_Ty_r, bias_Tz_r
    Fx_r += wrench_aligned.wrench.force.x
    Fy_r += wrench_aligned.wrench.force.y
    Fz_r += wrench_aligned.wrench.force.z
    Tx_r += wrench_aligned.wrench.torque.x
    Ty_r += wrench_aligned.wrench.torque.y
    Tz_r += wrench_aligned.wrench.torque.z
    counter_r += 1
    if counter_r == 1000:
        bias_Fx_r = Fx_r / 1000.0
        bias_Fy_r = Fy_r / 1000.0
        bias_Fz_r = Fz_r / 1000.0
        bias_Tx_r = Tx_r / 1000.0
        bias_Ty_r = Ty_r / 1000.0
        bias_Tz_r = Tz_r / 1000.0
        bias_computed_r = 1
    return

def callback_left_arm(msg):
    wrench_aligned_l = msg
    # Compute bias for the first 1000 samples:
    if bias_computed_l == 0:
        get_bias_l(wrench_aligned_l)
        if counter_l == 1000:
            print 'L_ Bias Fx: ', bias_Fx_l
            print 'L_ Bias Fy: ', bias_Fy_l
            print 'L_ Bias Fz: ', bias_Fz_l
            print 'L_ Bias Tx: ', bias_Tx_l
            print 'L_ Bias Ty: ', bias_Ty_l
            print 'L_ Bias Tz: ', bias_Tz_l
    else:
        unbiased_Fx = wrench_aligned_l.wrench.force.x - bias_Fx_l
        unbiased_Fy = wrench_aligned_l.wrench.force.y - bias_Fy_l
        unbiased_Tx = wrench_aligned_l.wrench.torque.x - bias_Tx_l
        unbiased_Ty = wrench_aligned_l.wrench.torque.y - bias_Ty_l
        wrench_aligned_l.wrench.force.z -= bias_Fz_l
        wrench_aligned_l.wrench.torque.z -= bias_Tz_l
        wrench_aligned_l.wrench.force.x = unbiased_Fx*math.cos(alpha) - unbiased_Fy*math.sin(alpha)
        wrench_aligned_l.wrench.force.y = -(unbiased_Fx*math.sin(alpha)) - (unbiased_Fy*math.cos(alpha))
        wrench_aligned_l.wrench.torque.x = unbiased_Tx*math.cos(alpha) - unbiased_Ty*math.sin(alpha)
        wrench_aligned_l.wrench.torque.y = -(unbiased_Tx*math.sin(alpha)) - (unbiased_Ty*math.cos(alpha))
        pub_l.publish(wrench_aligned_l)
    return

def get_bias_l(wrench_aligned):
    global Fx_l, Fy_l, Fz_l, counter_l, bias_computed_l, bias_Fx_l, bias_Fy_l, bias_Fz_l
    global Tx_l, Ty_l, Tz_l, bias_Tx_l, bias_Ty_l, bias_Tz_l
    Fx_l += wrench_aligned.wrench.force.x
    Fy_l += wrench_aligned.wrench.force.y
    Fz_l += wrench_aligned.wrench.force.z
    Tx_l += wrench_aligned.wrench.torque.x
    Ty_l += wrench_aligned.wrench.torque.y
    Tz_l += wrench_aligned.wrench.torque.z
    counter_l += 1
    if counter_l == 1000:
        bias_Fx_l = Fx_l / 1000.0
        bias_Fy_l = Fy_l / 1000.0
        bias_Fz_l = Fz_l / 1000.0
        bias_Tx_l = Tx_l / 1000.0
        bias_Ty_l = Ty_l / 1000.0
        bias_Tz_l = Tz_l / 1000.0
        bias_computed_l = 1
    return

    
    
def transform_test():

    rospy.init_node('transform_test', anonymous=True)
    
##    rospy.Subscriber("/ft/r_gripper_motor", WrenchStamped, callback_right_arm)
    
    rospy.Subscriber("/ft/l_gripper_motor", WrenchStamped, callback_left_arm)
    ### Test left callback function with right arm topic
##    rospy.Subscriber("/ft/r_gripper_motor", WrenchStamped, callback_left_arm)
    
    rospy.spin()
    
if __name__ == '__main__':

    transform_test()

    try:
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()
            
    except KeyboardInterrupt:
        pass
##    raw_input("Press a key to continue")
##    print 'size of force_x', len(force_x)
##    print 'element [1] of force_x', force_x[1]
##    print 'element [1] of force_x_tf', force_x_tf[1]
    



