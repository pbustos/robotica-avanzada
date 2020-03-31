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
		self.Period = 2000
		self.timer.start(self.Period)

		# Connect to VREP IK
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]
		self.base = self.client.simxGetObjectHandle('gen3', self.client.simxServiceCall())[1]
		self.camera_arm = self.client.simxGetObjectHandle('Camera_Arm', self.client.simxServiceCall())[1]
		#self.imageVREP = TImage()

		self.Maq1FirtsMovement.start()
		self.destroyed.connect(self.t_dejarObjeto_to_finalizar)

		self.JOYSTICK_EVENT = False

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		return True

	def detectarObjetos(self):	
		#posicion de reconocimiento
		#self.client.simxSetObjectPose(self.target, self.base, [0.01, -0.35, 0.53, 0.0, 0.0, 0.0, 0.0], self.client.simxServiceCall())		
		
		# if self.JOYSTICK_EVENT:
		# 	#self.client.simxSetObjectPosition('target', self.client.simxServiceCall())[1]
		# 	self.JOYSTICK_EVENT = False

		# capture image
		res, resolution, imageVREP = self.client.simxGetVisionSensorImage(self.camera_arm, False, self.client.simxServiceCall())
		img = np.fromstring(imageVREP, np.uint8).reshape( resolution[1],resolution[0], 3)
		img = cv2.flip(img, 0)
		cv2.imshow("Gen3",img)
		cv2.waitKey(1)
		
		listaObjetos = self.getAprilTags(img, resolution)
		if listaObjetos:
			l = listaObjetos[0]
			target = self.client.simxGetObjectPose(self.target, self.base, self.client.simxServiceCall())[1]
			#objActual = np.array([listaObjetos[0].tx, listaObjetos[0].ty, listaObjetos[0].tz]) / 1000.
			objActual = np.array([listaObjetos[0].cx, listaObjetos[0].cy])
			diffPos = np.array([320, 240.]) - objActual
			height = l.tz/1000.
			print(diffPos, "----" , height, target[2] )
			try:
				target[0] = target[0] + ( diffPos[0] * 0.0005 )
				target[1] = target[1] - ( diffPos[1] * 0.0005 )
				target[2] = target[2] - ( height * 0.05 )
				
				self.client.simxSetObjectPose(self.target, self.base, target, self.client.simxServiceCall())
			except:
				print("Sometginh went wrong")
				
			
	def moverBrazo(self):
		# mover hasta centrar sobre pieza
		#print self.target_tag


		# ejes x, z
		# self.client.simxSetObjectPose(self.target, self.base, self.listCoordenadasObjeto, self.client.simxServiceCall())
		pass


	def cogerObjeto(self):
		#cerramos las pinzas
		pass

	def dejarObjeto(self):
		#movemos el brazo a esa posicion objetivo
		self.client.simxSetObjectPose(self.target, self.base, self.listCoordenadasObjeto, self.client.simxServiceCall())
		#abrimos pinzas

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

# =============== Slots methods for State Machine ===================
# ===================================================================
	#
	# sm_inicializar
	#
	@QtCore.Slot()
	def sm_inicializar(self):
		print("Entered state inicializar")
		#self.client.simxSetObjectPose(self.target, self.base, [0.01, -0.35, 0.53, 0.0, 0.0, 180.0, 0.0], self.client.simxServiceCall())		
		self.t_inicializar_to_detectarObjetos.emit()
		pass

	#
	# sm_cogerObjeto
	#
	@QtCore.Slot()
	def sm_cogerObjeto(self):
		print("Entered state cogerObjeto")
		self.cogerObjeto()
		pass

	#
	# sm_dejarObjeto
	#
	@QtCore.Slot()
	def sm_dejarObjeto(self):
		print("Entered state dejarObjeto")
		self.dejarObjeto()
		pass

	#
	# sm_detectarObjetos
	#
	@QtCore.Slot()
	def sm_detectarObjetos(self):
		print("Entered state detectarObjetos")
		ready_to_godown = self.detectarObjetos()
		time.sleep(0.1)
		self.t_detectarObjetos_to_detectarObjetos.emit()
		
	# sm_moverBrazo
	#
	@QtCore.Slot()
	def sm_moverBrazo(self):
		print("Entered state moverBrazo")
		self.moverBrazo()
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

