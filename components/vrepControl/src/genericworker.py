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

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
	ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

additionalPathStr = ''
icePaths = [ '/opt/robocomp/interfaces' ]
try:
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
	icePaths.append('/opt/robocomp/interfaces')
except:
	print('SLICE_PATH environment variable was not exported. Using only the default paths')
	pass

ice_JoystickAdapter = False
for p in icePaths:
	if os.path.isfile(p+'/JoystickAdapter.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"JoystickAdapter.ice"
		Ice.loadSlice(wholeStr)
		ice_JoystickAdapter = True
		break
if not ice_JoystickAdapter:
	print('Couln\'t load JoystickAdapter')
	sys.exit(-1)
from RoboCompJoystickAdapter import *
ice_CameraRGBDSimplePub = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimplePub.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimplePub.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimplePub = True
		break
if not ice_CameraRGBDSimplePub:
	print('Couln\'t load CameraRGBDSimplePub')
	sys.exit(-1)
from RoboCompCameraRGBDSimplePub import *
ice_CameraRGBDSimple = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimple.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimple.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimple = True
		break
if not ice_CameraRGBDSimple:
	print('Couln\'t load CameraRGBDSimple')
	sys.exit(-1)
from RoboCompCameraRGBDSimple import *
ice_AprilTagsServer = False
for p in icePaths:
	if os.path.isfile(p+'/AprilTagsServer.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"AprilTagsServer.ice"
		Ice.loadSlice(wholeStr)
		ice_AprilTagsServer = True
		break
if not ice_AprilTagsServer:
	print('Couln\'t load AprilTagsServer')
	sys.exit(-1)
from RoboCompAprilTagsServer import *


from camerargbdsimplepubI import *
from joystickadapterI import *


class GenericWorker(QtCore.QObject):

	kill = QtCore.Signal()
#Signals for State Machine
	t_initialize_to_compute = QtCore.Signal()
	t_compute_to_compute = QtCore.Signal()
	t_compute_to_finalize = QtCore.Signal()

#-------------------------

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


		self.apriltagsserver_proxy = mprx["AprilTagsServerProxy"]

		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

#State Machine
		self.defaultMachine= QtCore.QStateMachine()
		self.compute_state = QtCore.QState(self.defaultMachine)
		self.initialize_state = QtCore.QState(self.defaultMachine)

		self.finalize_state = QtCore.QFinalState(self.defaultMachine)


#------------------
#Initialization State machine
		self.initialize_state.addTransition(self.t_initialize_to_compute, self.compute_state)
		self.compute_state.addTransition(self.t_compute_to_compute, self.compute_state)
		self.compute_state.addTransition(self.t_compute_to_finalize, self.finalize_state)


		self.compute_state.entered.connect(self.sm_compute)
		self.initialize_state.entered.connect(self.sm_initialize)
		self.finalize_state.entered.connect(self.sm_finalize)
		self.timer.timeout.connect(self.t_compute_to_compute)

		self.defaultMachine.setInitialState(self.initialize_state)

#------------------

#Slots funtion State Machine
	@QtCore.Slot()
	def sm_compute(self):
		print("Error: lack sm_compute in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_initialize(self):
		print("Error: lack sm_initialize in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_finalize(self):
		print("Error: lack sm_finalize in Specificworker")
		sys.exit(-1)


#-------------------------
	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print("Period changed", p)
		self.Period = p
		self.timer.start(self.Period)
