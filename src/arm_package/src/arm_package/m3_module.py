import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTabWidget, QSlider, QApplication, QPushButton, QAbstractSpinBox, QFrame, QGroupBox, QRadioButton, QDoubleSpinBox, QLCDNumber,QLineEdit
from python_qt_binding.QtCore import Qt
from std_msgs.msg import String, Float32
import ast

#standard lib ROS-QT


#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~---------ROS-QT Plugin template--------~~~~~~~~~~~~~----------------------------------------------#
#*******************************************************************************************************************************#
class M3(Plugin):
    def __init__(self, context):
        super(M3, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('M3')
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print ('arguments: ', args)
            print ('unknowns: ', unknowns)
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('arm_package'), 'resource', 'm3_gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('M3-controller')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~---------ROS Communication between the scrips--------~~~~~~~~~~~~~----------------------------------------------#
#*******************************************************************************************************************************#
        self.pub_gui_talker = rospy.Publisher('M3_Frontend', String, queue_size=10)
        rospy.Subscriber('M3_Backend', String, self.m_callback)
        self.rate=rospy.Rate(1)
        self.rate.sleep()
        self.b_motor_selection =0
#       #pub_gui_talker means that message goes from gui to talker
#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------Panel control Qt widgets--------~~~~~~~~~~~~~-----------------------------------#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------buttons, slider, radiobuttons etc.~~~~~~~~~~~-----------------------------------#
#*******************************************************************************************************************************#
        self._widget.Set_pos_sli.valueChanged.connect(self.m_position_updater) #- reading the values from the slider and passing to the method
        self._widget.Set_pos_sli.sliderPressed.connect(self.m_position_updater)
        self._widget.Auto_mode.toggled.connect(self.m_control_mode) # switch between manual and auto mode
        self._widget.Connect_button.clicked[bool].connect(self.m_connector) #
        self._widget.Disconnect_button.clicked[bool].connect(self.m_disconnect)
        self._widget.Refresh_button.clicked[bool].connect(self.m_refresher_button)
        self._widget.Always_update.clicked[bool].connect(self.m_refresher)
        self._widget.Emergency_Stop.clicked[bool].connect(self.m_emergency)
        self._widget.Position_gain.returnPressed.connect(self.m_setGains)
        self._widget.Damping_gain.returnPressed.connect(self.m_setGains)
        self._widget.Set_gains_button.clicked[bool].connect(self.m_setGains)
        self._widget.Home_button.clicked[bool].connect(self.m_Home)
        self._widget.Motor_selection.activated.connect(self.m_motor)

        self._widget.Always_update.setHidden(True)
        self._widget.Auto_mode.setHidden(True)
        self._widget.positioncontrol.toggled.connect(self.m_controller)
        self._widget.velocitycontrol.toggled.connect(self.m_controller)
        self._widget.torquecontrol.toggled.connect(self.m_controller)
        self._widget.positioncontrol.setHidden(True)
        self._widget.velocitycontrol.setHidden(True)
        self._widget.torquecontrol.setHidden(True)


    def m_controller(self):
        self.pub_gui_talker.publish("test")

#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------Methods for the GUI without any effect on ROS---------~~------------------------#
#*******************************************************************************************************************************#

    def m_control_mode (self): # switch for the auto/manual mode
        if self._widget.Auto_mode.isChecked() == True: #(security check if auto mode is activate)
            self._widget.Set_pos_sli.setDisabled(True) #(to avoid the use with the manual mode)
            self._widget.Set_pos_sli.setHidden(True) #(to avoid the use with the manual mode)
            self._widget.Position_gain.setHidden(True) #(to avoid the use with the manual mode)
            self._widget.Damping_gain.setHidden(True) #(to avoid the use with the manual mode)
            self._widget.Set_gains_button.setDisabled(True) #(to avoid the use with the manual mode)
            self.m_Auto_mode()
        else:
            self._widget.Set_pos_sli.setEnabled(True) #(to activate the manual mode again)
            self._widget.Set_pos_sli.setVisible(True) #(to activate the manual mode again)
            self._widget.Position_gain.setVisible(True)#(to activate the manual mode again)
            self._widget.Damping_gain.setVisible(True)#(to activate the manual mode again)
            self._widget.Set_gains_button.setEnabled(True)#(to activate the manual mode again)



#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------Methods for the GUI with any effect on ROS-~~~~~--------------------------------#
#*******************************************************************************************************************************#

    def m_Home(self): # Home position of motor
        self.pub_gui_talker.publish("send_home;")

    def m_motor(self):
        motion_controller = self._widget.Motor_selection.currentText()
        if (motion_controller == "Motor1"):
            self.b_motor_selection=1
            self.pub_gui_talker.publish("Motor;"+str(self.b_motor_selection))
        else:
            self.b_motor_selection = 2
            self.pub_gui_talker.publish("Motor;"+str(self.b_motor_selection))



    def m_connector(self): # To connect the motor
        self.pub_gui_talker.publish("motor_init;")

    def m_device_updater (self,data):
        self._widget.Encoder_cpr.setText(str(data['encoder_cpr']))
        self._widget.Pole_Pairs.setText(str(data['pole_pairs']))

    def m_refresher_button (self):
        self.pub_gui_talker.publish("refresh")

    def m_refresher(self): # to update the values of encoder and Temperature
        if (self._widget.Always_update.isChecked())==True:
            self.pub_gui_talker.publish("refresh")
            self.m_position_updater()

    def m_setGains(self): # setting the gains for the motor Position and velocity
        Position_gain = (self._widget.Position_gain.text())
        Damper_gain = (self._widget.Damping_gain.text())
        self.pub_gui_talker.publish("set_Position_gain;"+str(Position_gain))
        self.pub_gui_talker.publish("set_Damper_gain;"+str(Damper_gain))

    def m_disconnect (self):# to disconnect the motor (Communication between can and computer still on but it can't send/read message)
        self.pub_gui_talker.publish("send_disconnect;")

    def m_emergency (self):# Stop function
        self.pub_gui_talker.publish("send_emergency_signal;")

    def m_position_updater (self): #------------------to change an update the values of the Target_Position
        s_now_pos= (self._widget.Set_pos_sli.value()) # getting the actual value of slider
        self.pub_gui_talker.publish("slider_pos;"+str(s_now_pos))

    def m_Auto_mode(self): #configuration of Automatic mode
        if (self._widget.Auto_mode.isChecked()) == True: #(security check if auto mode is activate)
            self.pub_gui_talker.publish('Auto_mode_on;')

#*******************************************************************************************************************************#
#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------ROS Callbacks---------~~~~~~~~~~~~~----------------------------------~~~~~~~~~~-#
#*******************************************************************************************************************************#
#*******************************************************************************************************************************#


    def m_callback (self,data):
# read the whole message
        rospy.loginfo(rospy.get_caller_id() + "I heard in gui %s", data.data)

#divide the message and only call if you read startswith(ROS callback)
        if(data.data.startswith("Home;")):
            self._widget.Set_pos_sli.setValue(0)
            #state=data.data.split(";")[1]
            #self._widget.Enconder_pos_display.display(state)
#
        elif(data.data.startswith('disconnected;')):
            self._widget.Set_pos_sli.setValue(0)
            #m_motor()
            if(data.data.startswith('Values_in_degrees;')):
                state=data.data.split(";")[1]
                if (self.b_motor_selection == 1):
                    self._widget.Enconder_pos_display.display(str(state))
                elif(self.b_motor_selection == 2):
                    self._widget.Enconder_pos_display_2.display(str(state))


        elif (data.data.startswith("slider_pos_update;")):
            Setposval_deg=data.data.split(";")[1]
            if (self.b_motor_selection == 1):
                self._widget.Is_position_display.display(str(Setposval_deg))
            elif (self.b_motor_selection == 2):
                self._widget.Is_position_display_2.display(str(Setposval_deg))

        elif (data.data.startswith("slider_enc_update;")):
            Encoderval_deg=data.data.split(";")[1]
            self.m_motor()
            #Encoderval_deg=ast.literal_eval(Encoderval_deg) # data remains as dictionry
            if (self.b_motor_selection == 1):
                self._widget.Enconder_pos_display.display(str(Encoderval_deg))
            elif(self.b_motor_selection == 2):
                self._widget.Enconder_pos_display_2.display(str(Encoderval_deg))

        elif (data.data.startswith("velocity;")):
            velocity=data.data.split(";")[1]
            if (self.b_motor_selection == 1):
                self._widget.velocity.setText(str(velocity))
            elif(self.b_motor_selection == 2):
                self._widget.velocity_2.setText(str(velocity))
            #Encoderval_deg=ast.literal_eval(Encoderval_deg) # data remains as dictionry
            #self._widget.velocity.setText(str(velocity))

        elif (data.data.startswith("torque;")):
            torque=data.data.split(";")[1]
            self.m_motor()
            if(self.b_motor_selection == 1):
                self._widget.torque.setText(str(torque))
            elif(self.b_motor_selection == 2):
                self._widget.torque_2.setText(str(torque))
            #Encoderval_deg=ast.literal_eval(Encoderval_deg) # data remains as dictionry

        elif(data.data.startswith('Connected;')):
            self.m_setGains()# setting the values for the gains

        elif(data.data.startswith('motor_info;')):
            d_Motor_info=data.data.split(";")[1]
            if (self.b_motor_selection == 1):
                self._widget.calibrated.setText('Yes')
            elif(self.b_motor_selection == 2):
                self._widget.calibrated_2.setText('Yes')


        elif(data.data.startswith('device_info;')):
            d_Device_info = data.data.split(";")[1]
            if (d_Device_info == 1):
                self._widget.Deviceid.setText(d_Device_info)
            elif(d_Device_info == 2):
                self._widget.Deviceid_2.setText(d_Device_info)

        elif(data.data.startswith("refreshed;")):
            self.m_refresher()
            self.rate.sleep()


#*******************************************************************************************************************************#
#----------~~~~~~~~~~~~~~~~~~~~~~~~-------------Standard rqt functions----------~~~~~~~~~~~~~-----------------------------------#
#*******************************************************************************************************************************#

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrMyPlugininsic configuration, usually using:
        # v = instance_settings.value(k)
        pass










































































































































































#Easteregg

#REALLY Sorry if you have to read this code (I had only three days for it)
