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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <stdlib.h>
#include <mutex>
#define TIEMPO_MAX 100
#define DIST_MIN 200

class SpecificWorker : public GenericWorker
{
  //Implementar nueva estructura con 4 campos (id,x,z y marca de tiempo q nos indique si esta a la vista o no)
Q_OBJECT
public:
	
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void newAprilTag(const tagsList &tags);

public slots:
	void compute(); 	

private:
   int estado = 0;
   float tagX,tagZ;//Coordenadas reales de la AprilTag en la sala
  // int tiempoTotal = 0; //Cuenta el tiempo que ha estado iterando (cada iteracion se incrementa)
  QVec tagInWorld;
  struct Tag{
    QMutex mutex;
    float x,z;//Distancia robot - tag
    int id,tiempo;
    /*Introducir coordenadas distancia robot - tag
     */
    void set(float x_, float z_)
    {
	  QMutexLocker ml(&mutex);
	  x = x_;
	  z = z_;
    }
    void setTiempo(int tiempo_)
    {
      QMutexLocker ml(&mutex);
      tiempo = tiempo_;
    }
    void setId(int id_)
    {
      QMutexLocker ml(&mutex);
      id = id_;
    }
    float getX(){
     QMutexLocker ml(&mutex);
     return x;
    }
    float getZ(){
     QMutexLocker ml(&mutex);
     return z;
    }
    int getId(){
     QMutexLocker ml(&mutex);
     return id;
    }
    int getTiempo(){
    QMutexLocker ml(&mutex);
    return tiempo;
    }
  };
  Tag tag;
  
  InnerModel *innermodel;
  
};

#endif

