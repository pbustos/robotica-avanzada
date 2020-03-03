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

#import yolo
import RoboCompYoloServer as yolo

#import pyhton
import sys
import os
import time
import threading
import cv2

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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.Period = 20
		self.timer.start(self.Period)

		self.defaultMachine.start()
		self.destroyed.connect(self.t_compute_to_finalize)

		# connect
		self.router, self.transport, self.session_manager = bM.connect()
		# Create required services
		self.device_config = DeviceConfigClient(self.router)
		self.base = BaseClient(self.router)
		self.base_cyclic = BaseCyclicClient(self.router)

		self.joystickVector = {'x':0, 'y':0, 'z':0}
		bM.cartesian_Position_movement(self.base, self.base_cyclic, 0)

		#video capture
		self.video_capture = cv2.VideoCapture('rtsp://192.168.1.10/color')
		self.depth_capture = cv2.VideoCapture('rtsp://192.168.1.10/depth')

		self.display = True

	def __del__(self):
		print('SpecificWorker destructor')
		# disconnect
		bM.disconnect(self.session_manager, self.transport)

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		#bM.cartesian_Relative_movement(self.base, self.base_cyclic, self.joystickVector['x'], self.joystickVector['y'], self.joystickVector['z'], 0, 0, 0)
		
		#leemos las dos imagenes
		_, frame = self.video_capture.read()
		_, frameDepth = self.depth_capture.read()

		# resize
		#frame = cv2.resize(frame, (608,608))
		#frameDepth = cv2.resize(frameDepth, (608,608))

		#enviamos el contenido a yolo
		frame, box = self.callYolo(frame, frame.shape)

		#calculamos el promedio del rectangulo identificado
		posNuevaImagen.x = box.left
		posNuevaImagen.y = box.top
		posNuevaImagen.width = box.right - box.left
		posNuevaImagen.height = box.bot - box.top

		m = cv2.mean(frameDepth(posNuevaImagen))
		print(m)

		cv2.imshow("",frame)
	


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
		print("Entered state compute")
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
	# sendData
	#
	def JoystickAdapter_sendData(self, data):
		if hasattr(data,"buttons") and data.buttons[0].clicked == True:
				self.joystickVector['x'] = 0
				self.joystickVector['y'] = 0
				self.joystickVector['z'] = 0
				bM.cartesian_Position_movement(self.base, self.base_cyclic, 0)
		else: 
			if abs([axis.value for axis in data.axes if axis.name == "x"][0]) < 0.3:
				self.joystickVector['x'] = 0
			else:
				self.joystickVector['x'] = [axis.value for axis in data.axes if axis.name == "x"][0] / 10.0
			
			if abs([axis.value for axis in data.axes if axis.name == "y"][0]) < 0.3:
				self.joystickVector['y'] = 0
			else:
				self.joystickVector['y'] = [axis.value for axis in data.axes if axis.name == "y"][0] / 10.0 
			
			if abs([axis.value for axis in data.axes if axis.name == "z"][0]) < 0.3:
				self.joystickVector['z'] = 0
			else:
				self.joystickVector['z'] = [axis.value for axis in data.axes if axis.name == "z"][0] / 10.0

	def callYolo(self, image, res):
		try:
			yimg = yolo.TImage(width=res[0], height=res[1], depth=3, image=image)
			objects = self.yoloserver_proxy.processImage(yimg)
			print(len(objects))
			if self.display:	
				if len(objects)>0:
					for box in objects:
						print(box)
						if box.prob > 50:
							p1 = (box.left, box.top)
							p2 = (box.right, box.bot)
							offset = int((p2[1] - p1[1]) / 2)
							pt = (box.left + offset, box.top + offset) 
							cv2.rectangle(image, p1, p2, (0, 0, 255), 4)
							font = cv2.FONT_HERSHEY_SIMPLEX
							cv2.putText(image, box.name + " " + str(int(box.prob)) + "%", pt, font, 1, (255, 255, 255), 2)				
			return image, box
		except  Exception as e:
			print("error", e)



