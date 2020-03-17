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

import sys
import os
import time
import threading
import cv2
import numpy as np
import copy

#vrep
import vrep
import b0RemoteApi

#import api kortex
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, DeviceConfig_pb2, Session_pb2
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient

# Import the utilities helper module
sys.path.append('/utilities')
import utilities

#import librerias creadas
sys.path.append('/basicMovements')
import basicMovements as bM

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.Period = 50
		self.timer.start(self.Period)

		self.defaultMachine.start()
		self.destroyed.connect(self.t_compute_to_finalize)

		# Connect to Arm
		#self.router, self.transport, self.session_manager = bM.connect()

		# Create required services
		# self.device_config = DeviceConfigClient(self.router)
		# self.base = BaseClient(self.router)
		# self.base_cyclic = BaseCyclicClient(self.router)

		# Connect to VREP INNER-ARM
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.wall_camera = self.client.simxGetObjectHandle('Camera_Arm', self.client.simxServiceCall())
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]

		# Obtiene los elementos que son los actuadores
		self.actuators = []
		for i in range(7):
			self.actuators.append(self.client.simxGetObjectHandle('Actuator'+str(i+1), self.client.simxServiceCall())[1])

		# Move arm to [0 0 0]
		self.joystickVector = {'x':0, 'y':0, 'z':0}
		#bM.cartesian_Position_movement(self.base, self.base_cyclic, 0)
		# MOVE ALSO THE INNER-ARM

		self.innerYolo = []
		self.outerYolo = []
		
		self.DISPLAY = True
		self.JOYSTICK_EVENT = False
		self.VREP =  True

	def __del__(self):
		print('SpecificWorker destructor')
		# disconnect
		#bM.disconnect(self.session_manager, self.transport)

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
	
		'''
		if self.JOYSTICK_EVENT:
			bM.cartesian_Relative_movement(self.base, self.base_cyclic, self.joystickVector['x'], self.joystickVector['y'], self.joystickVector['z'], 0, 0, 0)
			self.JOYSTICK_EVENT = False
		
		if self.VREP:
			res, resolution, imageVREP = self.client.simxGetVisionSensorImage(self.wall_camera[1],False, self.client.simxServiceCall())
			imageVREP, objectsVREP = self.callYoloVREP(imageVREP, resolution)
			cv2.imshow("Gen3",imageVREP)

		#image brazo fisico
		imageBrazo, objectsBrazo = self.callYolo(self.frame, self.frame.shape)
		'''

		# Read current ARM joint values and INNER-ARM joints accordingly
		#currentActuatorPosition = bM.getActuator(self.base, self.base_cyclic)
		#for i in range(len(currentActuatorPosition)):
		#	self.client.simxSetJointPosition(self.actuators[i], np.deg2rad(currentActuatorPosition[i]), self.client.simxServiceCall())
		
		# Check if two new images + yolo have come
		# Compute the difference between Yolo lists
		listResta = restaListaYolo(self)
		# Move, Add or Remove objects from INNER-MODEL


		#calculamos el promedio del rectangulo identificado
		# posNuevaImagen.x = box.left
		# posNuevaImagen.y = box.top
		# posNuevaImagen.width = box.right - box.left
		# posNuevaImagen.height = box.bot - box.top
		#m = cv2.mean(frameDepth(posNuevaImagen))
		#print(m)

		#cv2.imshow("Gen3",self.nueva)
		#cv2.waitKey(1)
	
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

# SUBSCRIPTION FROM JOYSTICK ==================================
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

	## SUBSCRIPTION FROM CameraSimpleYoloPub ==================================
	#
	def CameraRGBDSimpleYoloPub_pushRGBDYolo(self, image, depth, objs):
		#print ("New image ", len(image.image), image.cameraID)	
		if image.cameraID == 1:
			self.innerYolo = copy.deepcopy(objs)
		else:
			self.outerYolo = copy.deepcopy(objs)


# ################################################################################
# ###  metodos auxiliares
# ################################################################################
from PySide2 import QtGui as qt

def restaListaYolo(self):
	listResta = []

	print(self.innerYolo)
	if self.innerYolo ==[]:
		return self.outerYolo
	elif self.outerYolo ==[]:
		return self.innerYolo
	else:
		for objInner in self.innerYolo:
			for objOuter in self.outerYolo:
				if objInner.name != objOuter.name:
					continue
				else:
					print('hi')
					polygonInner = qt.QPolygon(qt.QRect(qt.QPoint(objInner.left, objInner.top), qt.QPoint(objInner.right, objInner.bot)))
					polygonOuter = qt.QPolygon(qt.QRect(qt.QPoint(objOuter.left, objOuter.top), qt.QPoint(objOuter.right, objOuter.bot)))
					if(polygonInner.intersects()):
						pass
					else: #si no intersecta 
						listResta.append(objOuter) #introducimos el outer porque lo estamos comparando con el inner
		return listResta


# ################################################################################
# ###  Yolo proxy
# ################################################################################
# 	def callYolo(self, image, res):
# 		try:
# 			yimg = yolo.TImage(width=res[0], height=res[1], depth=3, image=image)
# 			objects = self.yoloserver_proxy.processImage(yimg)
# 			if self.DISPLAY:	
# 				if len(objects)>0:
# 					print("Objects Brazo:", len(objects))
# 					for box in objects:
# 						#print(box)
# 						if box.prob > 50:
# 							p1 = (box.left, box.top)
# 							p2 = (box.right, box.bot)
# 							offset = int((p2[1] - p1[1]) / 2)
# 							pt = (box.left + offset, box.top + offset) 
# 							cv2.rectangle(image, p1, p2, (0, 0, 255), 4)
# 							font = cv2.FONT_HERSHEY_SIMPLEX
# 							cv2.putText(image, box.name + " " + str(int(box.prob)) + "%", pt, font, 1, (255, 255, 255), 2)				
# 			return image, objects
# 		except  Exception as e:
# 			print("error", e)

# 	def callYoloVREP(self, image, res):
# 		try:
# 			img = np.fromstring(image, np.uint8).reshape( res[1],res[0], 3)
# 			img = cv2.resize(img, (608, 608))
# 			resu = img.shape
# 			yimg = yolo.TImage(width=resu[0], height=resu[1], depth=3, image=img)
# 			objects = self.yoloserver_proxy.processImage(yimg)
# 			if self.DISPLAY:	
# 				if len(objects)>0:
# 					print("Objects VREP", len(objects))
# 					for box in objects:
# 						if box.prob > 50:
# 							p1 = (box.left, box.top)
# 							p2 = (box.right, box.bot)
# 							offset = int((p2[1] - p1[1]) / 2)
# 							pt = (box.left + offset, box.top + offset) 
# 							cv2.rectangle(img, p1, p2, (0, 0, 255), 4)
# 							font = cv2.FONT_HERSHEY_SIMPLEX
# 							cv2.putText(img, box.name + " " + str(int(box.prob)) + "%", pt, font, 1, (255, 255, 255), 2)				
# 			return img, objects
# 		except  Exception as e:
# 			print("error", e)



