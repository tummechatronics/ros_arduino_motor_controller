#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class trajectory():

    def __init__(self):
        rospy.init_node("trajectory_gene",anonymous=True)
        rospy.Subscriber('trajectory',String,self.m_callback)
        rate = rospy.Rate(2) #10hz
        self.pub_m2_1 = rospy.Publisher('M2_1_Frontend', String, queue_size=10)
        self.pub_m2_2 = rospy.Publisher('M2_2_Frontend', String, queue_size=10)
        self.pub_m3 = rospy.Publisher('M3_Frontend', String, queue_size=10)


    def m_circle(self):
        self.pub_m3.publish("Motor;"+str("1"))
#        for i in range (1,60):
        self.pub_m3.publish("slider_pos;"+str(60))
        self.pub_m3.publish("Motor;"+str("2"))
        #for i in range(1,50):
        self.pub_m3.publish("slider_pos;"+str(80))
        for i in range (1,3000):
            self.pub_m2_1.publish("slider_pos;"+str(3000))
            self.pub_m2_2.publish("slider_pos;"+str(3000))
    #    for i in range (2000,0,-1):
        self.pub_m2_1.publish("slider_pos;"+str(-1000))
        self.pub_m2_2.publish("slider_pos;"+str(-1000))


    def m_callback (self,data):
        #rospy.loginfo(rospy.get_caller_id()+" I heard in GUI %s",data.data)
        if(data.data.startswith("circle;")):
    	       self.m_circle()
        elif(data.data.startswith("rectangle;")):
    	       self.m_emergency_stop()
        elif(data.data.startswith("send_disconnect;")):
    	       self.m_disconnector()



if __name__ == '__main__':
    try:
        traj = trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
