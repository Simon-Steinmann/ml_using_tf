#! /usr/bin/env python

import rospy
from joint_pos_2_tf import Pos2Tf  
from  task_env_tf import j2n6s300TestEnv
        
        
if __name__ == "__main__":
    rospy.init_node('pos_2_tf_service_server')
    obj = j2n6s300TestEnv()
    for i in xrange(10000):
        obj.step(6)
    #rospy.spin()