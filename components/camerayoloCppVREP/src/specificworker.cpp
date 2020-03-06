/*
 *    Copyright (C)2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	defaultMachine.start();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

   	client = new b0RemoteApi("b0RemoteApi_c++Client","b0RemoteApiAddOn");
	qDebug() << __FUNCTION__ << "Connected";
	auto handle_hand_camera = client->simxGetObjectHandle("Camera_Arm", client->simxServiceCall());
	if( b0RemoteApi::readBool(handle_hand_camera, 0))
		hand_camera = b0RemoteApi::readInt(handle_hand_camera, 1);
	else
		qFatal("Error getting handle to hand-camera");
	auto handle_hand_target = client->simxGetObjectHandle("target", client->simxServiceCall());
	if( b0RemoteApi::readBool(handle_hand_target, 0))
		hand_target = b0RemoteApi::readInt(handle_hand_target, 1);
	else
		qFatal("Error getting handle to target");
	
		
	this->Period = period;
	timer.start(50);
	emit this->t_initialize_to_compute();

}

void SpecificWorker::compute()
{
	auto resImg = client->simxGetVisionSensorImage(hand_camera, false, client->simxServiceCall());
	//res, resolution, image
	auto resDepth = client->simxGetVisionSensorDepthBuffer(hand_camera, true, true, client->simxServiceCall());
	if(b0RemoteApi::readBool(resImg, 0))
		try
		{
			std::vector<int> size(2);
			b0RemoteApi::readIntArray(resImg, size, 1);
			RoboCompYoloServer::TImage image{ size[0], size[1], 3};
			qDebug() << size[0] << size[1];
			//cv::Mat yimg(size[1], size[0], CV8UC3, &b0RemoteApi::readByteArray(resImg, 2)[0]);
			//cv::resize(yimg, yimgdst, 608, 608, cv::INTER_LINEAR);
			image.image.resize(size[0]*size[1]*3);
			memcpy(&image.image[0], b0RemoteApi::readByteArray(resImg, 2).data(), size[0]*size[1]*3);
			//yoloserver_proxy->processImage(image);

			cv::Mat cvimg(size[1], size[0], CV_8UC3, b0RemoteApi::readByteArray(resImg, 2).data());
			cv::imshow("", cvimg);
			
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
	

	fps.print();
}


void SpecificWorker::sm_compute()
{
	//std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}






