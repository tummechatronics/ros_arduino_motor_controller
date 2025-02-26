#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# With this script you can generate trajectories for the 2.5D arm. Till now only A circle is implemented 
class trajectory():

    def __init__(self):
        rospy.init_node("trajectory_gene",anonymous=True)
        rospy.Subscriber('trajectory',String,self.m_callback)
        self.rate = rospy.Rate(10) #10hz
        self.pub_m2_1 = rospy.Publisher('M2_1_Frontend', String, queue_size=10)
        self.pub_m2_2 = rospy.Publisher('M2_2_Frontend', String, queue_size=10)
        self.pub_m3 = rospy.Publisher('M3_Frontend', String, queue_size=10)
        self.b_continue = False

    def m_top_left(self):
        self.pub_m3.publish("Motor;"+str("1"))
        self.pub_m3.publish("slider_pos;"+str(70))
        self.pub_m3.publish("Motor;"+str("2"))
        self.pub_m3.publish("slider_pos;"+str(60))
        self.pub_m2_2.publish("slider_pos;"+str(0))
        self.pub_m2_1.publish("slider_pos;"+str(3000))
    def m_top_right(self):
        self.pub_m3.publish("Motor;"+str("1"))
        self.pub_m3.publish("slider_pos;"+str(70))
        self.pub_m3.publish("Motor;"+str("2"))
        self.pub_m3.publish("slider_pos;"+str(60))
        self.pub_m2_1.publish("slider_pos;"+str(0))
        self.pub_m2_2.publish("slider_pos;"+str(5000))
    def m_bottom_left(self):
        self.pub_m3.publish("Motor;"+str("1"))
        self.pub_m3.publish("slider_pos;"+str(90))
        self.pub_m3.publish("Motor;"+str("2"))
        self.pub_m3.publish("slider_pos;"+str(50))
        self.pub_m2_1.publish("slider_pos;"+str(3000))
        self.pub_m2_2.publish("slider_pos;"+str(0))

    def m_bottom_right(self):
        self.pub_m3.publish("Motor;"+str("1"))
        self.pub_m3.publish("slider_pos;"+str(90))
        self.pub_m3.publish("Motor;"+str("2"))
        self.pub_m3.publish("slider_pos;"+str(50))
        self.pub_m2_1.publish("slider_pos;"+str(0))
        self.pub_m2_2.publish("slider_pos;"+str(5000))
    def m_reset(self):
        self.pub_m3.publish("Motor;"+str("1"))
        self.pub_m3.publish("slider_pos;"+str(0))
        self.pub_m3.publish("Motor;"+str("2"))
        self.pub_m3.publish("slider_pos;"+str(0))
        self.pub_m2_1.publish("slider_pos;"+str(0))
        self.pub_m2_2.publish("slider_pos;"+str(0))


    def m_callback (self,data):
        #rospy.loginfo(rospy.get_caller_id()+" I heard in GUI %s",data.data)
        if(data.data.startswith("top_left;")):

    	    self.m_top_left()

        elif(data.data.startswith("top_right;")):

    	    self.m_top_right()

        elif(data.data.startswith("bottom_left;")):

    	    self.m_bottom_left()

        elif(data.data.startswith("bottom_right;")):

            self.m_bottom_right()

        elif (data.data.startswith("circle;")):
            self.m_top_left()
            rospy.sleep(8)
            self.m_top_right()
            rospy.sleep(9)
            self.m_bottom_right()
            rospy.sleep(5)
            self.m_bottom_left()
            rospy.sleep(10)
            self.m_reset()




if __name__ == '__main__':
    try:
        traj = trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
