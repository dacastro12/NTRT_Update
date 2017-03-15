/*
 * Copyright © 2015, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */

/**
 * @file HipControlTFController.cpp
 * @brief Preferred Length Controller for HipControlTFModel
 * @author Dennis Castro
 * @version 1.0.0
 * $Id$
 */

// This module
#include "HipControlTFController.h"
// This application
#include "HipControlTFModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "controllers/tgTensionController.h"
//Bullet Physics
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <math.h>

# define M_PI 3.14159265358979323846 

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
HipControlTFController::HipControlTFController(const double initialLength, double timestep, btVector3 goalTrajectory) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    dt(timestep) {
    this->initPos = btVector3(0,0,0); 
    this->trajectory = btVector3(goalTrajectory.getX(),goalTrajectory.getY(),goalTrajectory.getZ());
}

//Fetch all the muscles and set their preferred length
void HipControlTFController::onSetup(HipControlTFModel& subject) {
	this->m_totalTime=0.0;
        this->initPos = endEffectorCOM(subject);
    const double flexion_length = 12;
    //const double brachioradialis_length = 12;
    //const double anconeus_length        = 6;
    //const double supportstring_length   = 0.5;

	const std::vector<tgBasicActuator*> flexion = subject.find<tgBasicActuator>("flexion");
	//const std::vector<tgBasicActuator*> anconeus        = subject.find<tgBasicActuator>("anconeus");
	//const std::vector<tgBasicActuator*> brachioradialis = subject.find<tgBasicActuator>("brachioradialis");
	//const std::vector<tgBasicActuator*> supportstrings  = subject.find<tgBasicActuator>("support");

    for (size_t i=0; i<flexion.size(); i++) {
		tgBasicActuator * const pMuscle = flexion[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(flexion_length, dt);
    }
 /*                                       
    // using for loops to anticipate more muscle fibers in the future
    for (size_t i=0; i<anconeus.size(); i++) {
		tgBasicActuator * const pMuscle = anconeus[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(anconeus_length, dt);
    }
     
    for (size_t i=0; i<brachioradialis.size(); i++) {
		tgBasicActuator * const pMuscle = brachioradialis[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(brachioradialis_length, dt);
    }
    
    for (size_t i=0; i<supportstrings.size(); i++) {
		tgBasicActuator * const pMuscle = supportstrings[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(supportstring_length, dt);
        cout << "string " << i << "\n";
    }
*/
}

// Set target length of each muscle, then move motors accordingly
void HipControlTFController::onStep(HipControlTFModel& subject, double dt) {
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;
    
    setFlexionTargetLength(subject, dt); //pitch
 //   setAnconeusTargetLength(subject, dt);        //yaw
    moveAllMotors(subject, dt);
    //updateActions(dt);

    btVector3 ee = endEffectorCOM(subject);
    std::cout << m_totalTime << " " << ee.getX() << " " << ee.getY() << " " << ee.getZ() << std::endl;
}
 
void HipControlTFController::setFlexionTargetLength(HipControlTFModel& subject, double dt) {
    const double mean_flexion_length = 12; //TODO: define according to vars
    double newLength = 5;
    const double amplitude    = mean_flexion_length/1;
    //const double angular_freq = 2;
    //const double phase = 0;
    const double dcOffset     = mean_flexion_length;
    const std::vector<tgBasicActuator*> flexion = subject.find<tgBasicActuator>("flexion");

    for (size_t i=0; i<flexion.size(); i++) {
		tgBasicActuator * const pMuscle = flexion[i];
		assert(pMuscle != NULL);
        //cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        newLength = dcOffset - amplitude*m_totalTime/5;
        if(newLength < dcOffset/3) {
            newLength = dcOffset/3;
        }
//Total time was 10
        if(m_totalTime > 5) {
            m_totalTime = 0;
        }

        //std::cout<<"calculating flexion target length:" << newLength << "\n";
        //std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        //cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}
/*
void ScarrArmController::setAnconeusTargetLength(ScarrArmModel& subject, double dt) {
    const double mean_anconeus_length = 6; //TODO: define according to vars
    double newLength = 0;
    const double amplitude = mean_anconeus_length/1;
    const double angular_freq = 2;
    const double phaseleft = 0;
    const double phaseright = phaseleft + M_PI;
    const double dcOffset = mean_anconeus_length;
    const std::vector<tgBasicActuator*> anconeusleft = subject.find<tgBasicActuator>("right anconeus");
    const std::vector<tgBasicActuator*> anconeusright = subject.find<tgBasicActuator>("left anconeus");

    for (size_t i=0; i<anconeusleft.size(); i++) {
        tgBasicActuator * const pMuscle = anconeusleft[i];
        assert(pMuscle != NULL);
        if(m_totalTime > 5) {
            newLength = amplitude * sin(angular_freq * m_totalTime + phaseleft) + dcOffset;
        } else {
            newLength = dcOffset;
        }
        pMuscle->setControlInput(newLength, dt);
    }

    for (size_t i=0; i<anconeusright.size(); i++) {
        tgBasicActuator * const pMuscle = anconeusright[i];
        assert(pMuscle != NULL);
        if(m_totalTime > 5) {
            newLength = amplitude * sin(angular_freq * m_totalTime + phaseright) + dcOffset;
        } else {
            newLength = dcOffset;
        }
        pMuscle->setControlInput(newLength, dt);
    } 
}

*/

//Move motors for all the muscles
void HipControlTFController::moveAllMotors(HipControlTFModel& subject, double dt) {
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
     
}

// Get actions from evolutionAdapter, transform them to this structure, and apply them
void HipControlTFController::updateActions(HipControlTFModel& subject, double dt) {
	/*vector<double> state=getState();
	vector< vector<double> > actions;

	//get the actions (between 0 and 1) from evolution (todo)
	actions=evolutionAdapter.step(dt,state);

	//transform them to the size of the structure
	actions = transformActions(actions);

	//apply these actions to the appropriate muscles according to the sensor values
    applyActions(subject,actions);
    */
}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > HipControlTFController::transformActions(vector< vector <double> > actions)
{
	double min=6;
	double max=12;
	double range=max-min;
	double scaledAct;
	for(unsigned i=0;i<actions.size();i++) {
		for(unsigned j=0;j<actions[i].size();j++) {
			scaledAct=actions[i][j]*(range)+min;
			actions[i][j]=scaledAct;
		}
	}
	return actions;
}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void HipControlTFController::applyActions(HipControlTFModel& subject, vector< vector <double> > act)
{
	//Get All the muscles of the subject
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	//Check if the number of the actions match the number of the muscles
	if(act.size() != muscles.size()) {
		cout<<"Warning: # of muscles: "<< muscles.size() << " != # of actions: "<< act.size()<<endl;
		return;
	}
	//Apply actions (currently in a random order)
	for (size_t i = 0; i < muscles.size(); ++i)	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		//cout<<"i: "<<i<<" length: "<<act[i][0]<<endl;
		pMuscle->setControlInput(act[i][0]);
	}
}

btVector3 HipControlTFController::endEffectorCOM(HipControlTFModel& subject) {
	const std::vector<tgRod*> endEffector = subject.find<tgRod>("endeffector");
	assert(!endEffector.empty());
	return endEffector[0]->centerOfMass();
}