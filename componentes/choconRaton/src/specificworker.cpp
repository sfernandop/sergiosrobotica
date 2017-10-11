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
    RoboCompLaser::TLaserData datosLaser;
    datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
    inner->updateTransformValues ( "base",bState.x,0,bState.z,0,bState.alpha,0 );

    switch ( estado )
    {
    case Estado::PARADO:
        if ( target.isEmpty() == false )
        {
            //Calcular punto inicial,punto final,y trayectoria entre ellos
            // parIni = //Falta el punto inicial
            parxz = target.get();//Punto final
            tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
            d = tR.norm2(); // distancia del robot al punto marcado
            estado= Estado::AVANZANDO;

        }
        break;
	
    case Estado::AVANZANDO:
      //Se recalcula la distancia segun avanza 
	tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
        d = tR.norm2(); // distancia del robot al punto marcado
	//Cada vez que avance, se van obteniendo los datos actualizados del laser
	datosLaser= laser_proxy->getLaserData();//Obtenemos datos del Laser
//	std::sort(datosLaser.begin()+20, datosLaser.end()-20,[](auto a, auto b){return a.dist< b.dist;});
	
        if ( d > 50 )
        {
	  
            //Si no ha llegado
            float velAvance = d;// version antigua
            float vRot = atan2 ( tR.x(),tR.z() ); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.

            if ( vRot > MAX_ROT )
            {
                vRot = MAX_ROT;
            }

            velAvance=MAX_ADV*sigmoide ( d ) *gauss ( vRot,0.3,0.5 );


            differentialrobot_proxy->setSpeedBase ( velAvance,vRot );
        }
        else
        {
            //Si ha llegado al sitio
            differentialrobot_proxy->setSpeedBase ( 0,0 ); //Se ParameterList
           
	    estado = Estado::LLEGADO;
        }

    
    break;
    
    
case Estado::GIRANDO:

    break;

case Estado::BORDEANDO:

    break;
case Estado::LLEGADO:
    target.setEmpty();
    estado=Estado::PARADO;
  
    break;

}

//    if ( target.isEmpty() == false )
//     {
//
//         std::pair<float, float> parxz = target.get();
//         QVec tR = inner->transform ( "base" ,QVec::vec3 ( parxz.first, 0, parxz.second ),"world" );
// 	float d = tR.norm2(); // distancia del robot al punto marcado
//
// 	if( d > 50 )//Si no ha llegado
// 	{
// 	  float velAvance = d;// version antigua
// 	  float vRot = atan2(tR.x(),tR.z()); //devuelve radianes del angulo q forma donde apunta el robot con el punto destino.
// 	/*  if (velAvance > MAX_ADV){
// 	    velAvance = MAX_ADV;
// 	  }*/
//
// 	  if( vRot > MAX_ROT){
// 	    vRot = MAX_ROT;
// 	  }
//
// 	  velAvance=MAX_ADV*sigmoide(d)*gauss(vRot,0.3,0.5);
//
//
// 	  differentialrobot_proxy->setSpeedBase(velAvance,vRot);
// 	  }
// 	else
// 	{ //Si ha llegado al sitio
// 	  differentialrobot_proxy->setSpeedBase(0,0);//Se ParameterList
// 	  target.setEmpty();
// 	}
//
//     }
// mtx.unlock();


}


void SpecificWorker::setPick ( const Pick &myPick )
{

    qDebug() <<  "x:" <<myPick.x;
    qDebug() <<  "y:" <<myPick.y;
    qDebug() <<  "z:" <<myPick.z;
    target.set ( myPick.x, myPick.z );
}

float SpecificWorker::gauss ( float vr, float vx, float h )
{
    float lambda=1.0;
    lambda= ( pow ( -vx,2.0 ) ) /log ( h );
    return pow ( E, ( pow ( -vr,2 ) ) /lambda );
}

float SpecificWorker::sigmoide ( float dis )
{
    return ( 1/ ( 1+pow ( E,-dis ) ) )-0.5;
}



