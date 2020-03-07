#
# Copyright (C) 2019 by YOUR NAME HERE
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
from PySide2.QtCore import QMutexLocker
import cv2
import math
import time
import numpy as np
import vrep
import b0RemoteApi
import RoboCompCameraRGBDSimple as RoboCompCameraRGBDSimple
import RoboCompYoloServer as RoboCompYoloServer

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		self.t_image = RoboCompCameraRGBDSimple.TImage()
		self.t_depth = TDepth()
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)
		self.contFPS = 0
		self.incs = [0,0,0]

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		print("Reading params")
		self.cameraid = int(params["cameraid"])  # range must be controlled 1..MAX_CAMERAS
		self.display = "true" in params["display"]
		self.callapriltags = "true" in params["callapriltags"]
		self.yolo = "true" in params["yolo"]
		self.initialize()
		self.count = 0
		return True

	def initialize(self):
		print("Initialize")
		self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApiAddOn')
		#self.wall_camera = self.client.simxGetObjectHandle('camera_' + str(self.cameraid) + '_rgb',self.client.simxServiceCall())
		self.wall_camera = self.client.simxGetObjectHandle('Camera_Arm', self.client.simxServiceCall())
		self.target = self.client.simxGetObjectHandle('target', self.client.simxServiceCall())[1]
		self.goHigh()
		self.start = time.time()

	@QtCore.Slot()
	def compute(self):
		res, resolution, image = self.client.simxGetVisionSensorImage(self.wall_camera[1],False, self.client.simxServiceCall())
		depth_res, depth_resolution, depth = self.client.simxGetVisionSensorDepthBuffer(self.wall_camera[1],True, True, self.client.simxServiceCall())

		if not res:
			return

		self.t_image.image = image
		self.t_image.width = resolution[0]
		self.t_image.height = resolution[1]
		self.t_image.depth = 3
		
		self.t_depth.depth = depth
		self.t_depth.width = resolution[0]
		self.t_depth.height = resolution[1]

#		self.camerargbdsimplepub_proxy.pushRGBD(self.t_image, TDepth())

		if self.callapriltags:
			self.getAprilTags(image, resolution)
		
		if self.yolo:
			yimg = self.callYolo(image, resolution)

		if self.display:
			self.displayImage(image, resolution)
		
		if time.time() - self.start > 1:
			print("FPS:", self.contFPS)
			self.start = time.time()
			self.contFPS = 0
		self.contFPS += 1

		if any(np.abs(self.incs) > 0.01):
			self.gotoPose(self.incs)
		#self.incs = [0,0,0]
		
		return True

###################################################################################################
	def gotoPose(self, incs):

		succ, pose = self.client.simxGetObjectPosition(self.target, -1, self.client.simxServiceCall())
		#print(pose)
		new_pose = [(0.01*incs[i])+pose[i] for i in range(len(pose))]
		#print(new_pose)
		self.client.simxSetObjectPosition(self.target, -1, new_pose, self.client.simxServiceCall())

	def goHome(self):
		home = [-2.1890029907226562, -1.8270002603530884, 0.9950007200241089]
		self.client.simxSetObjectPosition(self.target, -1, home, self.client.simxServiceCall())
		
	def goHigh(self):
		home = [-2.1890029907226562, -1.8270002603530884, 1.3]
		self.client.simxSetObjectPosition(self.target, -1, home, self.client.simxServiceCall())
	

###################################################################################################

	def callYolo(self, image, res):
		try:
			img = np.fromstring(image, np.uint8).reshape( res[1],res[0], 3)
			img = cv2.resize(img, (608, 608))
			resu = img.shape
			yimg = RoboCompYoloServer.TImage(width=resu[0], height=resu[1], depth=3, image=img)
			objects = self.yoloserver_proxy.processImage(yimg)
			print(len(objects))
			if self.display:	
				if len(objects)>0:
					for box in objects:
						if box.prob > 50:
							p1 = (box.left, box.top)
							p2 = (box.right, box.bot)
							offset = int((p2[1] - p1[1]) / 2)
							pt = (box.left + offset, box.top + offset) 
							cv2.rectangle(img, p1, p2, (0, 0, 255), 4)
							font = cv2.FONT_HERSHEY_SIMPLEX
							cv2.putText(img, box.name + " " + str(int(box.prob)) + "%", pt, font, 1, (255, 255, 255), 2)				
			return img
		except  Exception as e:
			print("error", e)
	
	def displayImage(self, image, resolution):
		img = np.fromstring(image, np.uint8).reshape( resolution[1],resolution[0], 3)
		img = cv2.flip(img, 0)
		img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		cv2.drawMarker(img, (int(resolution[0]/2), int(resolution[1]/2)),  (0, 0, 255), cv2.MARKER_CROSS, 100, 1);
		cv2.imshow("Camera_" + str(self.cameraid), img)
		cv2.waitKey(1)
	
	def getAprilTags(self, image, resolution):
		try:
			frame = Image()
			#frame.data = img.flatten()
			frame.data = image
			frame.frmt = Format(Mode.RGB888Packet, resolution[0], resolution[1], 3)
			frame.timeStamp = time.time()
			# 280 porque es la parte de negro que ocupa todo el png
			tags_list = self.apriltagsserver_proxy.getAprilTags(frame=frame, tagsize=280, mfx=462, mfy=462);
			if len(tags_list) > 0:
				dist = np.sqrt(tags_list[0].tx*tags_list[0].tx+tags_list[0].ty*tags_list[0].ty+tags_list[0].tz*tags_list[0].tz)
			else:
				dist = 0
			print(frame.timeStamp, tags_list,dist)
		except Ice.Exception as ex:
			print(ex)

# =============== STUB ==============================================
# ===================================================================
	#
	# getImage
	#
	def CameraRGBDSimple_getImage(self):
		#ml = QMutexLocker(self.mutex)
		#print("returning image", self.t_image.width, self.t_image.height, self.t_image.depth)
		return self.t_image
	#
	# getAll
	#
	def CameraRGBDSimple_getAll(self):
		return (self.t_image, self.t_depth)

	#
	# getDepth
	#
	def CameraRGBDSimple_getDepth(self):
		return self.t_depth

# ===================================================================
	#
	# SUBSCRIPTON to sendData from JoystickAdapter
	#
	def JoystickAdapter_sendData(self, data):
		#print(data)
		x = [axis.value for axis in data.axes if axis.name == "x"][0]
		y = [axis.value for axis in data.axes if axis.name == "y"][0]
		z = [axis.value for axis in data.axes if axis.name == "z"][0]
		self.incs = [-x, -y, z]

# ===================================================================



# res, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, self.camhandle, 0, vrep.simx_opmode_oneshot_wait)
		# if res is not 0:
		# 	sys.exit()

		#resD, resolutionD, depth = vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.camDhandle, vrep.simx_opmode_buffer)
		
		# imgD = np.array(depth, dtype = np.float32)
		# imgD.resize([resolutionD[1], resolutionD[0], 1])
		# imgD = cv2.resize(imgD, (0, 0), None, .5, .5)
		# #imgD = np.rot90(imgD,2)
		# #imgD = np.fliplr(imgD)
		# imgD = cv2.normalize(imgD, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
		# imgDD = cv2.cvtColor(imgD, cv2.COLOR_GRAY2BGR)

		# horizontal_concat = np.concatenate((img, imgDD), axis=1)

		#cv2.imshow("ALab_CameraD_0", horizontal_concat)
