/*
 *    Copyright (C) 2018 by YOUR NAME HERE
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
#include "genericworker.h"
/**
* \brief Default constructor
*/
GenericWorker::GenericWorker(MapPrx& mprx) :
#ifdef USE_QTGUI
Ui_guiDlg()
#else
QObject()
#endif

{
	getapriltags_proxy = (*(GetAprilTagsPrx*)mprx["GetAprilTagsProxy"]);
	laser_proxy = (*(LaserPrx*)mprx["LaserProxy"]);
	differentialrobot_proxy = (*(DifferentialRobotPrx*)mprx["DifferentialRobotProxy"]);
	jointmotor_proxy = (*(JointMotorPrx*)mprx["JointMotorProxy"]);

	topicmanager_proxy = (*(IceStorm::TopicManagerPrx*)mprx["topicManager"]);


	mutex = new QMutex(QMutex::Recursive);

	#ifdef USE_QTGUI
		setupUi(this);
		show();
	#endif
	Period = BASIC_PERIOD;
	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));
	connect(&storm_timer, SIGNAL(timeout()), this, SLOT(check_storm()));
	storm_timer.start(storm_period);


// 	timer.start(Period);
}

/**
* \brief Default destructor
*/
GenericWorker::~GenericWorker()
{

}
void GenericWorker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void GenericWorker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	Period = p;
	timer.start(Period);
}


void GenericWorker::check_storm()
{
	try {
		topicmanager_proxy->ice_ping();
	} catch(const Ice::Exception& ex) {
		cout <<"Exception: STORM not running: " << ex << endl;
	}
}
