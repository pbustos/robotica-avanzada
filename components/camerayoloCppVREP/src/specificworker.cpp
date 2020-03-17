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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ShowImage");
		SHOW_IMAGE = (par.value == "true");
		par = params.at("Yolo");
		YOLO = (par.value == "true");
		par = params.at("Depth");
		DEPTH = (par.value == "true");
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
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
	std::vector<int> size;
	RoboCompYoloServer::Objects objs;	
	RoboCompYoloServer::TImage image;
	RoboCompCameraRGBDSimple::TDepth depth;
	RoboCompCameraRGBDSimple::TImage fimage;
	auto resImg = client->simxGetVisionSensorImage(hand_camera, false, client->simxServiceCall());
	if(b0RemoteApi::readBool(resImg, 0))
	{
		b0RemoteApi::readIntArray(resImg, size, 1);
		int cols = size[0]; int rows = size[1]; int len = cols*rows;
		image.width = cols; image.height = rows; image.depth = 3; image.image.resize(len);
		memcpy(&image.image[0], b0RemoteApi::readByteArray(resImg, 2).data(), len);
		if(YOLO)
		{
			try
			{
				objs = yoloserver_proxy->processImage(image);
				qDebug() << "objects" << objs.size();
			}
			catch(const Ice::Exception &e){	std::cout << e.what() << '\n';	}
		}

		// We need to swap the image in YoloServer::TImage to the type in RoboCompCameraRGBDSimple
		fimage.cameraID = 1;
		fimage.width = cols; fimage.height = rows; fimage.depth = 3; fimage.focalx = 500; fimage.focaly = 500; fimage.alivetime = 0; 
		fimage.image.resize(len); // inner camera's id is 1
		std::swap(fimage.image, image.image);  // swap from YoloServer::TImage image type

		if( SHOW_IMAGE )
		{
			cv::Mat cvimg = cv::Mat(cv::Size{640,480}, CV_8UC3,  b0RemoteApi::readByteArray(resImg, 2).data() );
			cv::imshow("", cvimg);
			cv::waitKey(1);
		}
	}
	else
		qDebug() << __FUNCTION__ << "Error capturing image";
	
	if(DEPTH)
	{
		auto resDepth = client->simxGetVisionSensorDepthBuffer(hand_camera, true, true, client->simxServiceCall());
		if( b0RemoteApi::readBool(resDepth, 0)) 
		{
			b0RemoteApi::readIntArray(resDepth, size, 1);
			int dcols = size[0]; int drows = size[1]; int dlen = dcols*drows*4;  // OJO float size
			depth.cameraID = 0;
			depth.width = dcols; depth.height = drows; depth.focalx = 500; depth.focaly = 500; depth.alivetime = 0; 
			depth.depth.resize(dlen); 
			memcpy(&depth.depth[0], b0RemoteApi::readByteArray(resDepth, 2).data(), dlen);
		}
		else
			qDebug() << __FUNCTION__ << "Error capturing depth";
	}
	try
	{ 
		//camerargbdsimpleyolopub_pubproxy->pushRGBDYolo(fimage, depth, objs);
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
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






