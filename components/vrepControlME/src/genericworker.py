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
ice_CameraRGBDSimpleYoloPub = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimpleYoloPub.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimpleYoloPub.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimpleYoloPub = True
		break
if not ice_CameraRGBDSimpleYoloPub:
	print('Couln\'t load CameraRGBDSimpleYoloPub')
	sys.exit(-1)
from RoboCompCameraRGBDSimpleYoloPub import *
ice_YoloServer = False
for p in icePaths:
	if os.path.isfile(p+'/YoloServer.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"YoloServer.ice"
		Ice.loadSlice(wholeStr)
		ice_YoloServer = True
		break
if not ice_YoloServer:
	print('Couln\'t load YoloServer')
	sys.exit(-1)
from RoboCompYoloServer import *


from camerargbdsimpleyolopubI import *
from joystickadapterI import *


class GenericWorker(QtCore.QObject):

	kill = QtCore.Signal()
#Signals for State Machine
	t_inicializar_to_detectarObjetos = QtCore.Signal()
	t_inicializar_to_moverBrazoToObj = QtCore.Signal()
	t_detectarObjetos_to_detectarObjetos = QtCore.Signal()
	t_detectarObjetos_to_moverBrazoToObj = QtCore.Signal()
	t_detectarObjetos_to_cogerObjeto = QtCore.Signal()
	t_moverBrazoToObj_to_detectarObjetos = QtCore.Signal()
	t_moverBrazoToObj_to_moverBrazoToObj = QtCore.Signal()
	t_moverBrazoToObj_to_cogerObjeto = QtCore.Signal()
	t_moverBrazoToObj_to_soltarObjeto = QtCore.Signal()
	t_cogerObjeto_to_cogerObjeto = QtCore.Signal()
	t_cogerObjeto_to_moverBrazoToObj = QtCore.Signal()
	t_cogerObjeto_to_soltarObjeto = QtCore.Signal()
	t_cogerObjeto_to_trasladarObjToPosFinal = QtCore.Signal()
	t_trasladarObjToPosFinal_to_trasladarObjToPosFinal = QtCore.Signal()
	t_trasladarObjToPosFinal_to_cogerObjeto = QtCore.Signal()
	t_trasladarObjToPosFinal_to_soltarObjeto = QtCore.Signal()
	t_soltarObjeto_to_soltarObjeto = QtCore.Signal()
	t_soltarObjeto_to_moverBrazoToObj = QtCore.Signal()
	t_soltarObjeto_to_finalizar = QtCore.Signal()
	t_soltarObjeto_to_inicializar = QtCore.Signal()

#-------------------------

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


		self.apriltagsserver_proxy = mprx["AprilTagsServerProxy"]

		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)

#State Machine
		self.Maq1FirtsMovement= QtCore.QStateMachine()
		self.detectarObjetos_state = QtCore.QState(self.Maq1FirtsMovement)
		self.moverBrazoToObj_state = QtCore.QState(self.Maq1FirtsMovement)
		self.cogerObjeto_state = QtCore.QState(self.Maq1FirtsMovement)
		self.trasladarObjToPosFinal_state = QtCore.QState(self.Maq1FirtsMovement)
		self.soltarObjeto_state = QtCore.QState(self.Maq1FirtsMovement)
		self.inicializar_state = QtCore.QState(self.Maq1FirtsMovement)

		self.finalizar_state = QtCore.QFinalState(self.Maq1FirtsMovement)


#------------------
#Initialization State machine
		self.inicializar_state.addTransition(self.t_inicializar_to_detectarObjetos, self.detectarObjetos_state)
		self.inicializar_state.addTransition(self.t_inicializar_to_moverBrazoToObj, self.moverBrazoToObj_state)
		self.detectarObjetos_state.addTransition(self.t_detectarObjetos_to_detectarObjetos, self.detectarObjetos_state)
		self.detectarObjetos_state.addTransition(self.t_detectarObjetos_to_moverBrazoToObj, self.moverBrazoToObj_state)
		self.detectarObjetos_state.addTransition(self.t_detectarObjetos_to_cogerObjeto, self.cogerObjeto_state)
		self.moverBrazoToObj_state.addTransition(self.t_moverBrazoToObj_to_detectarObjetos, self.detectarObjetos_state)
		self.moverBrazoToObj_state.addTransition(self.t_moverBrazoToObj_to_moverBrazoToObj, self.moverBrazoToObj_state)
		self.moverBrazoToObj_state.addTransition(self.t_moverBrazoToObj_to_cogerObjeto, self.cogerObjeto_state)
		self.moverBrazoToObj_state.addTransition(self.t_moverBrazoToObj_to_soltarObjeto, self.soltarObjeto_state)
		self.cogerObjeto_state.addTransition(self.t_cogerObjeto_to_cogerObjeto, self.cogerObjeto_state)
		self.cogerObjeto_state.addTransition(self.t_cogerObjeto_to_moverBrazoToObj, self.moverBrazoToObj_state)
		self.cogerObjeto_state.addTransition(self.t_cogerObjeto_to_soltarObjeto, self.soltarObjeto_state)
		self.cogerObjeto_state.addTransition(self.t_cogerObjeto_to_trasladarObjToPosFinal, self.trasladarObjToPosFinal_state)
		self.trasladarObjToPosFinal_state.addTransition(self.t_trasladarObjToPosFinal_to_trasladarObjToPosFinal, self.trasladarObjToPosFinal_state)
		self.trasladarObjToPosFinal_state.addTransition(self.t_trasladarObjToPosFinal_to_cogerObjeto, self.cogerObjeto_state)
		self.trasladarObjToPosFinal_state.addTransition(self.t_trasladarObjToPosFinal_to_soltarObjeto, self.soltarObjeto_state)
		self.soltarObjeto_state.addTransition(self.t_soltarObjeto_to_soltarObjeto, self.soltarObjeto_state)
		self.soltarObjeto_state.addTransition(self.t_soltarObjeto_to_moverBrazoToObj, self.moverBrazoToObj_state)
		self.soltarObjeto_state.addTransition(self.t_soltarObjeto_to_finalizar, self.finalizar_state)
		self.soltarObjeto_state.addTransition(self.t_soltarObjeto_to_inicializar, self.inicializar_state)


		self.detectarObjetos_state.entered.connect(self.sm_detectarObjetos)
		self.moverBrazoToObj_state.entered.connect(self.sm_moverBrazoToObj)
		self.cogerObjeto_state.entered.connect(self.sm_cogerObjeto)
		self.trasladarObjToPosFinal_state.entered.connect(self.sm_trasladarObjToPosFinal)
		self.soltarObjeto_state.entered.connect(self.sm_soltarObjeto)
		self.inicializar_state.entered.connect(self.sm_inicializar)
		self.finalizar_state.entered.connect(self.sm_finalizar)

		self.Maq1FirtsMovement.setInitialState(self.inicializar_state)

#------------------

#Slots funtion State Machine
	@QtCore.Slot()
	def sm_detectarObjetos(self):
		print("Error: lack sm_detectarObjetos in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_moverBrazoToObj(self):
		print("Error: lack sm_moverBrazoToObj in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_cogerObjeto(self):
		print("Error: lack sm_cogerObjeto in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_trasladarObjToPosFinal(self):
		print("Error: lack sm_trasladarObjToPosFinal in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_soltarObjeto(self):
		print("Error: lack sm_soltarObjeto in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_inicializar(self):
		print("Error: lack sm_inicializar in Specificworker")
		sys.exit(-1)

	@QtCore.Slot()
	def sm_finalizar(self):
		print("Error: lack sm_finalizar in Specificworker")
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
