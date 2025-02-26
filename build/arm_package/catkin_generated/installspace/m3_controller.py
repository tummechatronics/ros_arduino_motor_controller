#!/usr/bin/env python3

#################################################################################################
# the can message is defined : [canid][motor status][position][velocity][gain P][gain D][Torque]#
# to test independenly for motor 1 :															#
# starting the motor "1;1;0;0;0;0;0"															#
# setting the position to zero "1;2;0;0;0;0;0"													#
# Testing the position control "1;5;10;0;0.01;30;0"												#
# Testing the velocity control "1;5;0;10;0;10;0"												#
# Testing the torque control "1;5;0;0;0;0;5"													#
#################################################################################################
import rospy
import can
import struct
import signal
import time
import sys
import random

from std_msgs.msg import String,Float32


def exit_gracefully(signum, frame):
	signal.signal(signal.SIGINT, original_sigint)
	sys.exit(1)
	signal.signal(signal.SIGINT, exit_gracefully)


class t_motor_can_api:

	def __init__(self):

#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~---------ROS Communication between backend and frontend-~~~~~~~~~----------------------------#
#*******************************************************************************************************************************#
		self.pub_talker_gui = rospy.Publisher('M3_Backend', String, queue_size=10)
		rospy.Subscriber('M3_Frontend', String, self.m_callback)#pub_talker_gui means that message goes from talker to gui

#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~---------T-motor Communication~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~----------------------------~~#
#*******************************************************************************************************************************#
		can_interface = "can0"
		numberOfTmotors = 3
		self.bus = can.interface.Bus(can_interface, bustype = "socketcan",bitrate = 1000000)
		self.motorstatus= []
		self.pos= []
		self.velocity = []
		self.torque= []
		self.status = []
		self.b_calibrated = False
		self.D_gain = 0
		self.P_gain = 0
		self.a_motor_id = []
		self.b_motor_selection = 0
		for i in range(numberOfTmotors):
			self.motorstatus.append(rospy.Publisher('t_motor/' + str(i) + '/motorstatus', Float32, queue_size = 1))
			self.pos.append(rospy.Publisher('t_motor/' + str(i) + '/position', Float32, queue_size = 1))
			self.velocity.append(rospy.Publisher('t_motor/' + str(i) + '/velocity', Float32, queue_size = 1))
			self.torque.append(rospy.Publisher('t_motor/' + str(i) + '/torque', Float32, queue_size = 1))
			self.status.append(0)
			self.s_pub_motor_backend = rospy.Publisher('t_motor/'+str(i), String, queue_size=1)
			rospy.Subscriber('t_motor/' + str(i) + '/', String, self.cb_motor)
			self.a_motor_id.append(i)
		rospy.init_node('t_motor_can', anonymous = True)
		self.nodename = rospy.get_name()
		rospy.loginfo("%s started" % self.nodename)

#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~---------T-motor functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~----------------------------~~#
#*******************************************************************************************************************************#
	def m_connect(self):
		for i in range (len(self.a_motor_id)):
			if i== 0:
				continue
			else:
				self.s_pub_motor_backend.publish(str(self.a_motor_id[i])+";"+"1;0;0;0.01;30;0")
				#str(self.a_motor_id[i])
		#self.s_pub_motor_backend.publish('1;1;0;0;0.01;30;0')
				#if (self.b_calibrated==False):
				#self.s_pub_motor_backend.publish('0;1;0;0;0.01;30;0')
				self.s_pub_motor_backend.publish(str(self.a_motor_id[i])+";"+"2;0;0;0;0;0")
				self.pub_talker_gui.publish("motor_info;"+str(self.a_motor_id[i]))
				self.pub_talker_gui.publish("device_info;"+str(self.a_motor_id[i]))
		#rospy.loginfo("connected")
		#self.b_calibrated = True
	def m_emergency_stop(self):
		for i in range (len(self.a_motor_id)):
			if i== 0:
				continue
			else:
				self.s_pub_motor_backend.publish(str(self.a_motor_id[i])+';0;0;0;0;0;0')
		#rospy.loginfo("Stopping motor")
		#self.bus.send(state)
	def m_disconnector(self):
		#self.s_pub_motor_backend.publish(self.a_motor_id[i]+';5;0;0;0.01;30;0')
		self.m_emergency_stop()
		rospy.loginfo("disconnected")

	def m_motor_selection(self,s_motor_select):
		if (s_motor_select == "1"):
			self.b_motor_selection = 1
            #self.pub_gui_talker.publish("Motor;"+str(self.b_motor_selection))
		else:
            #self.b_motor_selection = 1
            #self.pub_gui_talker.publish("Motor;"+str(self.b_motor_selection))
			self.b_motor_selection = 2

	def float_to_uint(self,number,v_min,v_max,bits):
		number = number/(v_max-v_min)*2**bits
		if (v_min != 0):
			number = int(number + 2**bits/2)
		else:
			number = int(number)
		return number

	def cb_motor(self,data):
		values = data.data.split(';')
		#rospy.loginfo(values)
		if (int(values[1]) == 1):
			msg = can.Message(arbitration_id=0x0+int(values[0]), data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])
		elif (int(values[1]) == 0):
			msg = can.Message(arbitration_id=0x0+int(values[0]), data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD])
		elif (int(values[1]) == 2):
			msg = can.Message(arbitration_id=0x0+int(values[0]), data=[0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE])
		else:
			p_des = self.float_to_uint(float(values[2]),-95.5,95.5,16)
			v_des = self.float_to_uint(float(values[3]),-30,30,12)
			kp_des = self.float_to_uint(float(values[4]),0,5,12)
			kd_des = self.float_to_uint(float(values[5]),0,500,12)
			t_des = self.float_to_uint(float(values[6]),-18,18,12)
			data0 = p_des >>8
			data1 = p_des & 0xFF
			data2 = v_des >>4
			data3 = ((v_des & 0xF)<<4)|(kp_des >>8)
			data4 = kp_des & 0xFF
			data5 = kd_des >> 4
			data6 = ((kd_des & 0xF)<<4)|(t_des >> 8)
			data7 = t_des & 0xFF
			msg = can.Message(arbitration_id=0x0+int(values[0]), data=[data0,data1,data2,data3,data4,data5,data6,data7])
		self.bus.send(msg)

	def m_refresh(self):
		for msg in self.bus:
			if (msg.arbitration_id == 0x00):
				nachricht = struct.unpack('BBBBBB',msg.data)
				position=float((nachricht[1]<<8)|nachricht[2])
				velocity = float((nachricht[3]<<4)|(nachricht[4]>>4))
				torque = float(((nachricht[4] &  0x0F) << 8)|nachricht[5])
				position = position - 0xFFFF/2
				velocity = velocity - 0xFFF/2
				torque = torque -0xFFF/2
				position = position / (0xFFFF/2) * 95.5
				velocity = velocity / (0xFFF/2) * 30
				torque = torque / (0xFFF/2) * 18
				#self.pos[nachricht[0]].publish(position)
				#self.velocity[nachricht[0]].publish(velocity)
				#self.torque[nachricht[0]].publish(torque)
				self.pub_talker_gui.publish("slider_enc_update;"+str(position))
				self.pub_talker_gui.publish("velocity;"+str(velocity))
				self.pub_talker_gui.publish("torque;"+str(torque))
				#rospy.loginfo("refreshed")
				break;


	def m_callback (self,data):
		rospy.loginfo(rospy.get_caller_id()+" I heard in GUI %s",data.data)
		if(data.data.startswith("motor_init;")):
			self.m_connect()
		elif(data.data.startswith("send_emergency_signal;")):
			self.m_emergency_stop()
		elif(data.data.startswith("send_disconnect;")):
			self.m_disconnector()
		elif(data.data.startswith("refresh")):
			self.m_refresh()
		elif(data.data.startswith("Motor;")):
			Motor=data.data.split(";")[1]
			self.m_motor_selection(Motor)
			#rospy.loginfo("refreshed")
		elif(data.data.startswith("slider_pos;")):
			Position=data.data.split(";")[1]
			self.s_pub_motor_backend.publish(str(self.b_motor_selection)+";5;"+str(Position)+";0;"+ str(self.P_gain)+";"+str(self.D_gain)+";0")
			self.pub_talker_gui.publish("slider_pos_update;"+str(Position))
			self.m_refresh()
		elif(data.data.startswith("send_home;")):
			self.s_pub_motor_backend.publish(str(self.b_motor_selection)+";5;0;0;0.01;30;0")
			self.m_refresh()
			self.pub_talker_gui.publish("Home;")
		elif(data.data.startswith("set_Position_gain;")):
			self.P_gain = data.data.split(";")[1]
		elif(data.data.startswith("set_Damper_gain;")):
			self.D_gain = data.data.split(";")[1]



if __name__ == '__main__':
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, exit_gracefully)
	tma = t_motor_can_api()
	rospy.spin()













































#Easteregg

#REALLY Sorry if you have to read this code (I had only three days for it)
