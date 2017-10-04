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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
 target.x = 0;
 target.y = 0;
 target.z = 0;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
    inner = new InnerModel ( "/home/salabeta/robocomp/files/innermodel/simpleworld.xml" );
    timer.start ( Period );
    target.empty = true;
   
    return true;
}

void SpecificWorker::compute()
{
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState ( bState );
   
    inner->updateTransformValues ( "base",bState.x,0,bState.z,0,bState.alpha,0 );
 
    if ( target.isEmpty() == false )
    {
        std::pair<float, float> parxz = target.get(); 
        QVec tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" ); //tR- Posicion del robot desde el pv del mundo 
	float d = tR.norm2(); // distancia del robot al punto marcado 
	
	if( d > 50 )//Si no ha llegado
	{
	  float velAvance = d;
	  if (velAvance > MAX_ADV)
	    velAvance = MAX_ADV;
	  float velRot = atan2(tR.x(),tR.z()); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.
	  if( velRot > MAX_ROT)
	    velRot = MAX_ROT;
 	   differentialrobot_proxy->setSpeedBase(velAvance,velRot);
        }
	else
	{ //Si ha llegado al sitio
	  differentialrobot_proxy->setSpeedBase(0,0);//Se ParameterList
	  target.setEmpty(); 
	}

    }
 // mtx.unlock();


}


void SpecificWorker::setPick ( const Pick &myPick )
{
   
    qDebug() <<  "x:" <<myPick.x;
    qDebug() <<  "y:" <<myPick.y;
    qDebug() <<  "z:" <<myPick.z;
    target.set(myPick.x, myPick.z);
  }




