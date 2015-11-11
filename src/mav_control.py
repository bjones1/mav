#!/usr/bin/env python
# .. -*- coding: utf-8 -*-
#
# The first line above allows Unix to run this program. In
# particular:
#
# * ``#!`` identifies this file as a script Unix can run.
# * ``/usr/bin/env`` tells Unix to look for then run
#   a program in order to execute this script. See the
#   `env manual page <http://linux.die.net/man/1/env>`_
#   for a bit more info.
# * ``python`` specifies which program env should search
#   for.
#
# So, overall, this tells Unix to look for Python, then use
# it to run this script.
#
# The second line of this program tells Python that this
# file is written in Unicode using the most common 8-bit
# encoding. See `PEP 263
# <https://www.python.org/dev/peps/pep-0263/>`_ for details
# on the syntax of this statement, or the `utf-8
# <https://en.wikipedia.org/wiki/UTF-8>`_ page for
# background on Unicode and encodings.
#
# ***************************************************
# mav_control.py - Top-level control for the ARDrone.
# ***************************************************
# This program allows you to repsond to user clicks on the
# GUI you designed. Running it is a two-step process:
#
# 1. Launch the AR Drone drivers to connect to and
#    communicate with the drone. At a terminal, type
#    ``roslaunch ardrone_autonomy ardrone.launch``.
#    Keep an eye on this window; it will tell you if it
#    loses its drone connection.
# 2. Run this program. One method: ``rosrun iamgirl
#    mav_control.py``.
#
# Imports
# =======
# First, we need to include some other Python `modules
# <https://docs.python.org/2/tutorial/modules.html>`_
# which provide the ability to send commands to our drone.
# The ``import`` statement accomplishes this; see modules_
# page for more information.
#
# Imports are listed in the order prescribed by `PEP 8
# <http://www.python.org/dev/peps/pep-0008/#imports>`_.
#
# Library imports
# ---------------
# None needed.
#
# Third-party imports
# -------------------
# None needed.
#
# Local imports
# -------------
# ButtonGui runs our GUI and ROS and displays video.
from mav_control_base import ButtonGui
# main starts up the GUI, telling it to use the class
# below to do so.
from mav_control_base import main
# Must import after ``mav_control_base`` to get SIP API set
# correctly.
from PyQt4.QtCore import QElapsedTimer, pyqtSlot
# For calling services wihch take no parameters: toggle
# camera and flat trim.
from std_srvs.srv import Empty as EmptyServiceType
# For calling services which take parameters.
from ardrone_autonomy.srv import CamSelect, \
  FlightAnim, LedAnim, RecordEnable
from time import sleep

count = 0

# creates an enum for the auto flying state machine
class Enum(frozenset):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

# defines the "state" for each auto flying state
_MAV_STATES = Enum( ('Takeoff', 'Find_Target', 'Increase_Alt', 'Fly_Over', 'Land') )

# The class below groups together the code and data used
# to tell the MAV what to do based on user GUI clicks.
# ``MavControl`` is the name of the class we're defining
# below. ``(ButtonGui)`` causes this class to add to the
# existing code in ButtonGui, which takes =care of all the
# lower-level work (displaying video, running the GUI, etc.)
class MavControl(ButtonGui):
        
    def on_hsThreshold_valueChanged(self, value):
        print(value)

    # This is called when the pbPressed button is pressed.
    # Naming is similar for other functions.
    def on_pbTakeoff_pressed(self):
        print("TAKE OFF!")
        self.controller.SendTakeoff()

    def on_pbLand_pressed(self):
        print("LAND")
        self.controller.SendLand()
        
    def on_pbFlatTrim_pressed(self):
        print("Flat trim")
        self.controller.SetFlatTrim()
        
    def on_pbEmergency_pressed(self):
        print("Reset")
        self.controller.SendEmergency()
        
    def on_pbLED_pressed(self):
        print("Animation #", self.dialAnimation.value())
        self.controller.SetLedAnimation(self.dialAnimation.value(), 5, 3)

    def on_pbCameraToggle_pressed(self):
        print("Camera toggle")
        self.controller.ToggleCamera()
    
    def on_pbUp_pressed(self):
        print("Up")
        self.controller.SetCommand(roll=0, pitch=0,
          yaw_velocity=0, z_velocity=0.5)
        self.pbTest.setValue(self.pbTest.value() + 10)

    def on_pbUp_released(self):
        print("Up done.")
        self.controller.hover()
        
    def on_pbDown_pressed(self):
        print("Down")
        self.controller.SetCommand(roll=0, pitch=0,
          yaw_velocity=0, z_velocity=-0.5)

    def on_pbDown_released(self):
        print("Down done.")
        self.controller.hover()
        
    def on_pbForward_pressed(self):
        print("Forward")
        self.controller.SetCommand(roll=0, pitch=0.5,
          yaw_velocity=0, z_velocity=0)

    def on_pbForward_released(self):
        print("Forward done.")
        self.controller.hover()
        
    def on_pbBackward_pressed(self):
        print("Backward")
        self.controller.SetCommand(roll=0, pitch=-0.5,
          yaw_velocity=0, z_velocity=0)

    def on_pbBackward_released(self):
        print("Backward done.")
        self.controller.hover()
        
    def on_pbLeft_pressed(self):
        print("Left")
        self.controller.SetCommand(roll=0.5, pitch=0,
          yaw_velocity=0, z_velocity=0)

    def on_pbLeft_released(self):
        print("Left done.")
        self.controller.hover()
        
    def on_pbRight_pressed(self):
        print("Right")
        self.controller.SetCommand(roll=-0.5, pitch=0,
          yaw_velocity=0, z_velocity=0)

    def on_pbRight_released(self):
        print("Right done.")
        self.controller.hover()
        
    def on_pbRotateLeft_pressed(self):
        print("Rotate Left")
        self.controller.SetCommand(roll=0, pitch=0,
          yaw_velocity=0.5, z_velocity=0)

    def on_pbRotateLeft_released(self):
        print("Rotate left done.")
        self.controller.hover()
        
    def on_pbRotateRight_pressed(self):
        print("Rotate Right")
        self.controller.SetCommand(roll=0, pitch=0,
          yaw_velocity=-0.5, z_velocity=0)

    def on_pbRotateRight_released(self):
        print("Rotate right done.")
        self.controller.hover()

    @pyqtSlot(bool)
    def on_cbAuto_clicked(self,
      # True is the checkbox is checked; False if not.
      checked):

        if checked:
            # Initialize our state if we're just entering
            # auto mode.
            self.state = 1
            # Create a timer for use in ``fly()``.
            self.elapsedTimer = QElapsedTimer()
        
        else checked:
            self.controller.SendLand()
        
        # This code will never get run since the state of a checkbox will 
        # only be checked or unchecked
        # else:
        #     # Return to a hover when leaving auto mode.
        #     self.controller.hover()
        
    # This is only called when the Auto checkbox is checked.
    def fly(self,
      # The x coordinate of the center of the tracked area.
      # It ranges between 0 and ``self.lbVideo.width() - 1``.
      x_center,
      # The y coordinate of the center of the tracked area.
      # It ranges between 0 and ``self.lbVideo.height() - 1``.
      y_center,
      # The area, in pixels, of the tracked region.
      cont_area):

        # Clear the description of what this does.
        self.lbAuto.setText('')
        
        # Decide what to do based on the state.
        if self.state == 1:
            #print(cont_area)
            # Take off
            # ^^^^^^^^
            # The Auto checkbox was just checked. Take off
            # to start out mission.
            self.updateAutoLabel('Takeoff!')
            self.controller.SendTakeoff()
          
            # Measure time from takeoff.
            self.elapsedTimer.start()
        
            # Move to the next state, waiting for takeoff to
            # finish.
            self.state = 2
            
        elif self.state == 2:
            # Wait until take off completed
            # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            self.updateAutoLabel('Waiting')
            # Don't send any commands until we're flying.
            # So, wait 5 seconds then go to the next state.
            if self.elapsedTimer.elapsed() >= 5000:
                if (cont_area < 8500 and x_center < 125 and x_center >= 0):
                    #Fly Left
                    self.updateAutoLabel('Fly Left')
                    self.controller.SetCommand(roll=0.0625, pitch=0.0625, yaw_velocity=0, z_velocity=0)
                    
                elif (cont_area < 8500 and x_center > 175):
                    #Fly Right
                    self.updateAutoLabel('Fly Right')
                    self.controller.SetCommand(roll=-0.0625, pitch=0.0625, yaw_velocity=0, z_velocity=0)
                    
                elif (cont_area < 8500 and x_center >= 125 and x_center <= 175):
                    #flys forward
                    self.updateAutoLabel('Fly Forward')
                    self.controller.SetCommand(roll=0, pitch=0.0625, yaw_velocity=0, z_velocity=0)
                
                elif (x_center == -1):
                    #Rotate right
                    self.updateAutoLabel('Rotate Right')
                    self.controller.SetCommand(roll=0, pitch=0, yaw_velocity=-0.5, z_velocity=0)
                
                else:
                    #flys UP
                    self.controller.SetCommand(roll=0, pitch=0, yaw_velocity=0, z_velocity=0.5)
                    self.state = 3
                
        elif self.state == 3:
            self.updateAutoLabel('Fly Up')
            if (cont_area == 0):
                self.elapsedTimer.restart()
                self.state = 4
        
        elif self.state == 4:
                #Fly Forward
                self.updateAutoLabel('Get over it {}'.format(self.elapsedTimer.elapsed()))
                self.controller.SetCommand(roll=0, pitch=0.0625, yaw_velocity=0, z_velocity=0.0)
                if self.elapsedTimer.elapsed() >= 5000:
                    self.state = 5
        
        elif self.state == 5:
            # ...your ideas...
            self.updateAutoLabel('Landing')
            self.controller.SendLand()
            self.cbAuto.toggle()
        else:
            self.updateAutoLabel('Unknown state! Help!')

        # 1. Determine what to do by examining ``x_center``,
        #    ``y_center``, etc.
        #
        # 2. Explain what your code will do:
        #    ``self.updateAutoLabel('Flying up!')``.
        #
        # 3. Then do it, using something like:
        #    ``self.controller.SetCommand(roll, pitch,
        #    yaw_velocity, z_velocity)``, where you fill in
        #    numbers in place of ``roll``, ``pitch``, etc.
        #
        # A template for your code::
        #
        
     

    # Explain what the drone is doing in auto mode by
    # displaying strings telling its intentions.
    def updateAutoLabel(self,
      # A string to add to the explanation.
      s):
        self.lbAuto.setText(self.lbAuto.text() + s)


if __name__=='__main__':
    main(MavControl)
