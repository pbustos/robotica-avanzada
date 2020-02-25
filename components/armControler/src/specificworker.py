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

#import pyhton
import sys
import os
import time
import threading

#import api kortex
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, Base_pb2

# Import the utilities helper module
sys.path.append('/utilities')
import utilities

#import librerias creadas
sys.path.append('/basicMovements')
import basicMovements as bM

import argparse

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.Period = 2000
		self.timer.start(self.Period)

		self.defaultMachine.start()
		self.destroyed.connect(self.t_compute_to_finalize)
		
		error_callback = lambda kException: print("_________ callback error _________ {}".format(kException))
		transport = TCPTransport()
		router = RouterClient(transport, error_callback)
		transport.connect('192.168.1.10', 10000)

		# Create session
		session_info = Session_pb2.CreateSessionInfo()
		session_info.username = "admin"
		session_info.password = "admin"
		session_info.session_inactivity_timeout = 60000   # (milliseconds)
		session_info.connection_inactivity_timeout = 2000 # (milliseconds)

		print("Creating session for communication")
		session_manager = SessionManager(router)
		session_manager.CreateSession(session_info)
		print("Session created")

		# Create required services
		self.device_config = DeviceConfigClient(router)
		self.base = BaseClient(router)
		self.base_cyclic = BaseCyclicClient(router)




	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		
		# Example core
		success = True

		success &= bM.cartesian_Home_movement(self.base, self.base_cyclic)
		print("Move Home \n")

		success&= bM.cartesian_ParonamicView_movement(self.base, self.base_cyclic)

		success &= bM.cartesian_Especific_movement(self.base, self.base_cyclic, 0.57, 0.0, 0.43, 90.0, 0, 90)
		print("Move Home Robot KINOVA\n")

		success &= bM.cartesian_Relative_movement(self.base, self.base_cyclic, 0, 0, 0, 0, 0, -10)
		print("Girar mano \n")

		success &= bM.gripper_Close_All(self.base)
		print("Close All \n")

		success &= bM.gripper_Open_All(self.base)
		print("Open All \n")

		success &= bM.gripper_Relative_Aperture(self.base, 0.50)
		print("Open/close 0.50 \n")

		if success:
			print("Movimientos completados \n")
		else:
			print("Se han producido errores en los movientos \n")
		
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

