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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
  tag.setId(-1);
  tag.set(0,0);
  tag.setTiempo(0);
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
	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");

	return true;
}

void SpecificWorker::compute()
{
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
  innermodel->updateTransformValues("base",bState.x,0,bState.z,0,bState.alpha,0);
  
  tag.setTiempo(tag.getTiempo() + 1);//Cada ejecucion se incrementa
  switch(estado)
  {
    case 0 : //Empieza girando hasta que encuentre una AprilTag
      if (tag.getId() == 0 && tag.getTiempo()< TIEMPO_MAX)
      {//Si encuentra etiqueta 0 y lleva menos de 100 iteraciones sin verla 
	estado = 1;
	//Calcular posicion absoluta del tag 0
	tagInWorld = innermodel->transform("world", QVec::vec3(tag.x,0,tag.z),"base");//Cambiamos a SRef del mundo
	irobjetivo_proxy->go(tagInWorld.x(),tagInWorld.z());
      }
      else irobjetivo_proxy->turn(0.5);
      break;
    case 1 : //Direccion a la tag 0
      if ( irobjetivo_proxy->getState() < DIST_MIN )//Llegando a la etiqueta 0
      {
	tagInWorld = innermodel->transform("world", QVec::vec3(tag.x,0,tag.z),"base");//Cambiamos a SRef del mundo
	irobjetivo_proxy->go(tagInWorld.x(),tagInWorld.z());
	estado = 2;
      }
      else irobjetivo_proxy->go(tagInWorld.x(),tagInWorld.z());//Sigue yendo a la tag 0
      break;
    case 2 : 
      break;
    case 3:
      break;
    case 4 : 
      break;
      
  }
}


void SpecificWorker::newAprilTag(const tagsList &tags)
{
  tag.set(tags.data()->tx,tags.data()->tz);
  tag.setId(tags.data()-> id);
  tag.setTiempo(0);
}






