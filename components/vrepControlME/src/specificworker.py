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
		self.Period = 50
		self.timer.start(self.Period)

		# Connect to VREP IK
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]
		self.pivote = self.client.simxGetObjectHandle('PivoteApproach', self.client.simxServiceCall())[1]
		self.biela = self.client.simxGetObjectHandle('BielaApproach', self.client.simxServiceCall())[1]
		self.home = self.client.simxGetObjectHandle('InitApproach', self.client.simxServiceCall())[1]
		self.base = self.client.simxGetObjectHandle('gen3', self.client.simxServiceCall())[1]
		self.camera_arm = self.client.simxGetObjectHandle('camera_in_hand', self.client.simxServiceCall())[1]

		self.Maq1FirtsMovement.start()
		self.destroyed.connect(self.t_dejarObjeto_to_finalizar)

		self.JOYSTICK_EVENT = False
		print("Leaving Init")

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		return True

	def detectCircles(self, orig):
		gray = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
		# detect circles in the image

		circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.8, 100)
		# ensure at least some circles were found
		if circles is not None:
			print("Detected circle")
			# convert the (x, y) coordinates and radius of the circles to integers
			circles = np.round(circles[0, :]).astype("int")
			# loop over the (x, y) coordinates and radius of the circles
			for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
				cv2.circle(orig, (x, y), r, (0, 255, 0), 4)
				cv2.rectangle(orig, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			# show the output image
		return orig, circles

	def getAprilTags(self, image, resolution):
		try:
			frame = Image()
			frame.data = image.flatten()
			#frame.data = image.data
			frame.frmt = Format(Mode.RGB888Packet, resolution[0], resolution[1], 3)
			frame.timeStamp = time.time()
			tags_list = self.apriltagsserver_proxy.getAprilTags(frame=frame, tagsize=80, mfx=554, mfy=554)
			return tags_list
	
		except Ice.Exception as ex:
			print(ex)
			
	########################################################################
	def iniciarlizarBrazo(self):
		while True:
			self.client.simxCallScriptFunction("moveToObjectHandle@gen3", 1,self.home,self.client.simxServiceCall())
			PosActual = self.client.simxGetObjectPose(self.target, self.base, self.client.simxServiceCall())[1]
			PosObj = self.client.simxGetObjectPose(self.home, self.base, self.client.simxServiceCall())[1]
			result = np.abs(np.array(PosActual) - np.array(PosObj))
			if(result[0] < 0.05 and result[1] < 0.05 and result[2] < 0.05 ):
				break
		time.sleep(20)
	
	def detectarObjetos(self):
		# capture image
		res, resolution, imageVREP = self.client.simxGetVisionSensorImage(self.camera_arm, False, self.client.simxServiceCall())
		img = np.fromstring(imageVREP, np.uint8).reshape( resolution[1],resolution[0], 3)
		img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		img = cv2.flip(img, 0)
		img, self.circles = self.detectCircles(img)
		cv2.imshow("Gen3", img)
		cv2.waitKey(1)

		#hay alguna biela
		if self.circles is not None:
			return True
		else:
			return False
			
	def moverBrazo(self):
		while True:
			self.client.simxCallScriptFunction("moveToObjectHandle@gen3", 1,self.biela,self.client.simxServiceCall())
			PosActual = self.client.simxGetObjectPose(self.target, self.base, self.client.simxServiceCall())[1]
			PosObj = self.client.simxGetObjectPose(self.biela, self.base, self.client.simxServiceCall())[1]
			result = np.abs(np.array(PosActual) - np.array(PosObj))
			if(result[0] < 0.05 and result[1] < 0.05 and result[2] < 0.05 ):
				break
		time.sleep(20)
		return True


	def cogerObjeto(self):
		#cerramos las pinzas
		self.client.simxCallScriptFunction("closeGripper@gen3", 1,0,self.client.simxServiceCall())
		time.sleep(20)
		return True

	def dejarObjeto(self):
		while True:
			self.client.simxCallScriptFunction("moveToObjectHandle@gen3", 1,self.pivote,self.client.simxServiceCall())
			PosActual = self.client.simxGetObjectPose(self.target, self.base, self.client.simxServiceCall())[1]
			PosObj = self.client.simxGetObjectPose(self.pivote, self.base, self.client.simxServiceCall())[1]
			result = np.abs(np.array(PosActual) - np.array(PosObj))
			if(result[0] < 0.05 and result[1] < 0.05 and result[2] < 0.05 ):
				break
		time.sleep(20)
		#abrimos pinzas
		self.client.simxCallScriptFunction("openGripper@gen3", 1,0,self.client.simxServiceCall())
		time.sleep(20)
		return True

# =============== Slots methods for State Machine ===================
# ===================================================================
	#
	# sm_inicializar
	#
	@QtCore.Slot()
	def sm_inicializar(self):
		print("Entered state inicializar")
		self.iniciarlizarBrazo()
		self.t_inicializar_to_detectarObjetos.emit()
		pass

	#
	# sm_detectarObjetos
	#
	@QtCore.Slot()
	def sm_detectarObjetos(self):
		print("Entered state detectarObjetos")
		if not self.detectarObjetos():
			self.t_detectarObjetos_to_detectarObjetos.emit()
		else:
			self.t_detectarObjetos_to_moverBrazo.emit()
	
	# sm_moverBrazo
	#
	@QtCore.Slot()
	def sm_moverBrazo(self):
		print("Entered state moverBrazo")
		if not self.moverBrazo():
			self.t_moverBrazo_to_detectarObjetos.emit()
		else:
			self.t_moverBrazo_to_cogerObjeto.emit()

	#
	# sm_cogerObjeto
	#
	@QtCore.Slot()
	def sm_cogerObjeto(self):
		print("Entered state cogerObjeto")
		if self.cogerObjeto():
			self.t_cogerObjeto_to_dejarObjeto.emit()

		pass

	#
	# sm_dejarObjeto
	#
	@QtCore.Slot()
	def sm_dejarObjeto(self):
		print("Entered state dejarObjeto")
		if self.dejarObjeto():
			self.t_dejarObjeto_to_inicializar.emit()
		pass

	#
	# sm_finalizar
	#
	@QtCore.Slot()
	def sm_finalizar(self):
		print("Entered state finalizar")
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

