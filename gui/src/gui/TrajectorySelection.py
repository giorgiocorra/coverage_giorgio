import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import PyQt4.QtCore
import pyqtgraph as pg

# necessary to have gui as a client, asking controller to save data
# from python_qt_binding.QtCore import QTimer, Slot
# from python_qt_binding.QtCore import pyqtSlot


import numpy

# import analysis
# import utils
import subprocess
# from mavros.msg import OverrideRCIn
# from mocap.msg import QuadPositionDerived

from quad_control.msg import quad_state_and_cmd


# import services defined in quad_control
# SERVICE BEING USED: TrajDes_GUI
from quad_control.srv import *

from std_srvs.srv import Empty

import argparse


# # Relative path
# import sys
# # import os
# rospy.logwarn(sys.path)
# sys.path.append(os.path.join(os.path.dirname(__file__),'..','..'))
# # sys.path.append(os.path.join(os.path.dirname(__file__),'..','..','..'))
# # sys.path.append(os.path.join(os.path.dirname(__file__),'..','..','..','..'))
# # sys.path.append(os.path.join(os.path.dirname(__file__), '..','..','..','..','..'))
# rospy.logwarn(sys.path)



# to work with directories relative to ROS packages
from rospkg import RosPack
# determine ROS workspace directory
rp = RosPack()
# determine ROS workspace directory where data is saved
package_path = rp.get_path('quad_control')
# import sys
import sys
sys.path.insert(0, package_path)
# import trajectories dictionaries
from scripts.TrajectoryPlanner import trajectories_dictionary


class TrajectorySelectionPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


        super(TrajectorySelectionPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TrajectorySelectionPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TrajectorySelection.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('TrajectorySelectionUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO SET DESIRED TRAJECTORY
        self._widget.SetTrajectory.clicked.connect(self.SetTrajectory)


        self._widget.DefaultOption1.toggled.connect(self.DefaultOptions)
        self._widget.DefaultOption2.toggled.connect(self.DefaultOptions)
        self._widget.DefaultOption3.toggled.connect(self.DefaultOptions)

        # Planner buttons
        self._widget.planner_start_button.clicked.connect(self.planner_start)
        self._widget.planner_stop_button.clicked.connect(self.planner_stop)



        count = 0
        # for key,class_name in trajectories_dictionary.items():
        for key in trajectories_dictionary.trajectories_dictionary.keys():
            self._widget.listWidget.insertItem(count,key)
            count += 1 

        self._widget.listWidget.itemClicked.connect(self.__print_trajectory_message)
        self._widget.SetTrajectory2.clicked.connect(self.__get_new_trajectory_parameters)


    def __print_trajectory_message(self):
        rospy.logwarn("testing")
        rospy.logwarn(self._widget.listWidget.currentItem().text())
        
        selected_class_name = self._widget.listWidget.currentItem().text()
        selected_class      = trajectories_dictionary.trajectories_dictionary[selected_class_name]
        string              = selected_class.parameters_to_string()
        self._widget.TrajectoryMessageInput.setPlainText(string)

        string_offset_and_rotation = selected_class.offset_and_rotation_to_string()
        self._widget.MessageOffsetAndRotation.setPlainText(string_offset_and_rotation)

        # self._widget.TrajectoryMessageInput.setPlainText(self._widget.listWidget.currentItem().text())
        return 

    def __get_new_trajectory_parameters(self):
        rospy.logwarn(self._widget.TrajectoryMessageInput.toPlainText())

        selected_class_name = self._widget.listWidget.currentItem().text()
        selected_class      = trajectories_dictionary.trajectories_dictionary[selected_class_name]
        string              = self._widget.TrajectoryMessageInput.toPlainText()
        parameters          = selected_class.string_to_parameters(string)
        rospy.logwarn(parameters)

        string_offset_and_rotation = self._widget.MessageOffsetAndRotation.toPlainText()
        offset, rotation           = selected_class.string_to_offset_and_rotation(string_offset_and_rotation)

        rospy.logwarn(offset)
        rospy.logwarn(rotation)

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("/"+self.namespace+'TrajDes_GUI',1.0)
            
            try:
                SettingTrajectory = rospy.ServiceProxy("/"+self.namespace+'TrajDes_GUI', TrajDes_Srv)

                reply = SettingTrajectory(selected_class_name,offset,rotation,parameters)

                if reply.received == True:
                    # if controller receives message, we know it
                    # print('Trajectory has been set')
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException, e:
                rospy.logwarn('Proxy for service that sets desired trajectory FAILED')
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e   
            
        except:
            rospy.logwarn('Timeout for service that sets desired trajectory')
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True) 
            # print "Service not available ..."        
            pass           


        # self._widget.TrajectoryMessageInput.setPlainText(string)        

        return

    def DefaultOptions(self):

        # radius, period and height
        if self._widget.DefaultOption1.isChecked():
            r  = 0
            w  = 0
            z  = 0.6

        if self._widget.DefaultOption2.isChecked():
            r  = 0.5
            w  = 0.1
            z  = 0.6

        if self._widget.DefaultOption3.isChecked():
            r  = 1
            w  = 0.1
            z  = 0.6        
        if self._widget.DefaultOption4.isChecked():
            r  = 0.5
            w  = 0.2
            z  = 0.6                   


        # Default values for buttons
        self._widget.box_radius_circle.setValue(r)
        self._widget.box_omega_circle.setValue(w)
        self._widget.box_z_circle.setValue(z)


    #@Slot(bool)
    def SetTrajectory(self):

        # print(vars(self._widget))

        rospy.logwarn('currently testing')
        rospy.logwarn("/"+self.namespace+'TrajDes_GUI')

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service("/"+self.namespace+'TrajDes_GUI',1.0)
            
            try:
                SettingTrajectory = rospy.ServiceProxy("/"+self.namespace+'TrajDes_GUI', TrajDes_Srv)

                # self._widget.TrajSelect.currentIndex() is the tab number
                # first tab is 0
                # second tab is 1
                # ... 

                if self._widget.TrajSelect.currentIndex() == 0:
                   traj,offset,rotation,parameters = self.fixed_point()

                if self._widget.TrajSelect.currentIndex() == 1:
                    traj,offset,rotation,parameters = self.circle()

                rospy.logwarn(traj)

                reply = SettingTrajectory(traj,offset,rotation,parameters)


                if reply.received == True:
                    # if controller receives message, we know it
                    # print('Trajectory has been set')
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 


            except rospy.ServiceException, e:
                rospy.logwarn('Proxy for service that sets desired trajectory FAILED')
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                # print "Service call failed: %s"%e   
            
        except:
            rospy.logwarn('Timeout for service that sets desired trajectory')
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True) 
            # print "Service not available ..."        
            pass     


    def fixed_point(self):

        traj = 'StayAtRest'
        
        x = self._widget.box_x.value()
        y = self._widget.box_y.value()
        z = self._widget.box_z.value()
        offset = numpy.array([x,y,z])
        
        rotation = numpy.array([0.0,0.0,0.0])

        parameters = None

        return traj,offset,rotation,parameters        

    def circle(self):

        traj = 'DescribeCircle'
        x = self._widget.box_x_circle.value()
        y = self._widget.box_y_circle.value()
        z = self._widget.box_z_circle.value()
        offset = numpy.array([x,y,z])

        phi   = 0.0
        theta = self._widget.box_theta_circle.value()
        psi   = self._widget.box_psi_circle.value()
        rotation = numpy.array([phi,theta,psi])

        r = self._widget.box_radius_circle.value()
        # from revolutions per second to rad per second
        w = self._widget.box_omega_circle.value()*2*3.14
        parameters = numpy.array([r,w])

        return traj,offset,rotation,parameters


    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""

    # start planned trajectory
    def planner_start(self):
        try:
            rospy.wait_for_service('planner_start',1)
            start = rospy.ServiceProxy('planner_start',PlannerStart)

            start(self._widget.planner_edit.toPlainText())
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('could not start planned trajectory')

    # stop planned trajectory
    def planner_stop(self):
        try:
            rospy.wait_for_service('planner_stop',1)
            stop = rospy.ServiceProxy('planner_stop',Empty)

            stop()
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('could not start planned trajectory')
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    

