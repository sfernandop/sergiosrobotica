/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#include <stdlib.h>
#include <time.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}


void SpecificWorker::compute()
{
 // qDebug()<<"Hola";
  TLaserData data = laser_proxy->getLaserData();
  //differentialrobot_proxy->setSpeedBase(90,0.1);
  srand(time(NULL));
  
  static float orientacion=1;
  std::sort(data.begin()+20, data.end()-20,[](auto a, auto b){return a.dist< b.dist;});
  
 /* for(auto d:data){
    qDebug()<<d.dist + d.angle;
  } */
   if(data[20].dist<310)
   {
       float d= rand()%1000000+100000;
	differentialrobot_proxy->setSpeedBase(0,orientacion*0.75);
	usleep(d);
   }
  else
  {
      orientacion=0;
       while(orientacion==0)
       {
	orientacion=rand()%3-1;
       }
   differentialrobot_proxy->setSpeedBase(350,0);
   qDebug()<<orientacion;
  }
   //   qDebug()<<data.front().dist;

     
   
   
    

  }
  
  
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}








