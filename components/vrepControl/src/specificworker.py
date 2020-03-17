#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from genericworker import *
import cv2
import math
import time
import numpy as np
import vrep
import b0RemoteApi
import RoboCompCameraRGBDSimple as RoboCompCameraRGBDSimple

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		# Connect to VREP IK
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]

		self.DISPLAY = True
		self.JOYSTICK_EVENT = False

		self.Period = 50
		self.timer.start(self.Period)

		self.imageVREP = TImage()
		self.DISPlAY = True

		self.defaultMachine.start()
		self.destroyed.connect(self.t_compute_to_finalize)

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):

		# Check KoyStick
		if self.JOYSTICK_EVENT:
			print(self.joystickVector)
			self.JOYSTICK_EVENT = False
		
		# Compute apriltags
		try:
			frame = Image(data = self.imageVREP.image, 
						  frmt = Format(Mode.RGB888Packet,self.imageVREP.width, self.imageVREP.height, self.imageVREP.depth),
						  timeStamp = time.time())
			# 280 porque es la parte de negro que ocupa todo el png
			tags_list = self.apriltagsserver_proxy.getAprilTags(frame=frame, tagsize=280, mfx=462, mfy=462);	
		except Ice.Exception as ex:
			print(ex)

		if self.DISPlAY:
			img = np.fromstring(self.imageVREP.image, np.uint8).reshape( self.imageVREP.height, self.imageVREP.width, self.imageVREP.depth)
			cv2.imshow("Gen3", img)

		# Now ACT

		return True

# =============== Slots methods for State Machine ===================
# ===================================================================
	#
	# sm_initialize
	#
	@QtCore.Slot()
	def sm_initialize(self):
		print("Entered state initialize")
		self.t_initialize_to_compute.emit()
		pass

	#
	# sm_compute
	#
	@QtCore.Slot()
	def sm_compute(self):
		#print("Entered state compute")
		self.compute()
		pass

	#
	# sm_finalize
	#
	@QtCore.Slot()
	def sm_finalize(self):
		print("Entered state finalize")
		pass


# =================================================================
# =================================================================


	#
	# SUBSCRIPTION to pushRGBD method from CameraRGBDSimplePub interface
	#
	def CameraRGBDSimpleYoloPub_pushRGBDYolo(self, im, dep, objs):
		#print ("New image ", len(im.image), im.cameraID)	
		self.imageVREP = im

	#
	# SUBSCRIPTION to sendData method from JoystickAdapter interface
	#
	def JoystickAdapter_sendData(self, data):
		#print("Data", data)
		if hasattr(data,"buttons") and data.buttons[0].clicked == True:
				self.joystickVector['x'] = 0
				self.joystickVector['y'] = 0
				self.joystickVector['z'] = 0
				bM.cartesian_Position_movement(self.base, self.base_cyclic, 0)
		else: 
		
			if abs([axis.value for axis in data.axes if axis.name == "x"][0]) < 0.2:
				self.joystickVector['x'] = 0
			else:
				self.joystickVector['x'] = [axis.value for axis in data.axes if axis.name == "x"][0] / 10.0
			
			if abs([axis.value for axis in data.axes if axis.name == "y"][0]) < 0.2:
				self.joystickVector['y'] = 0
			else:
				self.joystickVector['y'] = [axis.value for axis in data.axes if axis.name == "y"][0] / 10.0 
			
			if abs([axis.value for axis in data.axes if axis.name == "z"][0]) < 0.2:
				self.joystickVector['z'] = 0
			else:
				self.joystickVector['z'] = [axis.value for axis in data.axes if axis.name == "z"][0] / 10.0
			
			self.JOYSTICK_EVENT = True

