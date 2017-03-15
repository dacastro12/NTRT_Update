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
 * @file HungControlTFController.cpp
 * @brief Preferred Length Controller for HungControlTFModel
 * @author Dennis Castro
 * @version 1.0.0
 * $Id$
 */

// This module
#include "HungControlTFController.h"
// This application
#include "HungControlTFModel.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <string>

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

// Packages for TotalEnergySpent function
#include "learning/Configuration/configuration.h"
#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "helpers/FileHelpers.h"


# define M_PI 3.14159265358979323846 

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
HungControlTFController::HungControlTFController(const double initialLength, double timestep, btVector3 goalTrajectory) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    dt(timestep) {
      this->initPos = btVector3(0,0,0); 
      this->trajectory = btVector3(goalTrajectory.getX(),goalTrajectory.getY(),goalTrajectory.getZ());
    }

//Fetch all the muscles and set their preferred length
void HungControlTFController::onSetup(HungControlTFModel& subject) {
	this->m_totalTime=0.0;
    const double flexion_length = 70;
    //const double brachioradialis_length = 12;
    //const double anconeus_length        = 6;
    //const double supportstring_length   = 0.5;
    double dt = 0.0001;
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
void HungControlTFController::onStep(HungControlTFModel& subject, double dt) {
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;

    setFlexionTargetLength(subject, dt); //pitch
 //   setAnconeusTargetLength(subject, dt);        //yaw
    moveAllMotors(subject, dt);
    //updateActions(dt);

    // EE Tracker
   // btVector3 ee = endEffectorCOM(subject);
   // std::cout << m_totalTime << " " << ee.getX() << " " << ee.getY() << " " << ee.getZ() << std::endl;
}
 
void HungControlTFController::setFlexionTargetLength(HungControlTFModel& subject, double dt) {
   const double mean_flexion_length = 100; //TODO: define according to vars
    double startLength = 75.5; 
    double MoE = 1000; //0.01GPa = 0.01 10^9 N/m^2 = 1000 N/cm^2 
    double tension = 0;
    double area = 0.01979; // area of cord in cm^2
    double newLength = 29;
    const double amplitude    = mean_flexion_length/1;
    //const double angular_freq = 2;
    //const double phase = 0;
    const double dcOffset = mean_flexion_length;
    const std::vector<tgBasicActuator*> flexion = subject.find<tgBasicActuator>("flexion");
    //const std::btVector3<DCModel*> stiffness = subject.find<DCModel>("c.stiffness");

    for (size_t i=0; i<flexion.size(); i++) {
		tgBasicActuator * const pMuscle = flexion[i];
		assert(pMuscle != NULL);
        // cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        newLength = dcOffset - amplitude*m_totalTime/5;
        tension = ((MoE*((pMuscle->getCurrentLength()) - startLength))/78)*area;
        if(newLength < dcOffset/3) {
            newLength = dcOffset/3;
		tension = ((MoE*((pMuscle->getCurrentLength()) - startLength))/78)*area;
        }

        if(m_totalTime > 5) {
            newLength = newLength + amplitude/3;
		tension = ((MoE*((pMuscle->getCurrentLength()) - startLength))/78)*area;
		if(m_totalTime >10){
			m_totalTime = 0;
		}
        }
        
	std::cout<< m_totalTime << " " << pMuscle->getCurrentLength() << " " << newLength << " " << tension << " " << pMuscle->getTension() << std::endl;
	// std::cout<<"calculating flexion target length:" << newLength << "\n";
	//std::cout<<"Tension:"<< tension << "\n";
	//std::cout<<"TensionP:"<< pMuscle->getTension()   <<"\n";
        // std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        // cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
//Need a reset timer or something to get it to work.
//  for (size_t i=5; i<flexion.size(); i++) {
//		tgBasicActuator * const pMuscle = flexion[i];
//		assert(pMuscle != NULL);
//        cout <<"t: " << pMuscle->getCurrentLength() << endl;
//        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
//        newLength = dcOffset + amplitude*m_totalTime/5;
//        if(newLength < dcOffset/3) {
//            newLength = dcOffset/3;
//        }
//
//        if(m_totalTime > 10) {
//            m_totalTime = 0;
//        }
    //    std::cout<<"calculating flexion target length:" << newLength << "\n";
  //      std::cout<<"m_totalTime: " << m_totalTime << "\n";
//		pMuscle->setControlInput(newLength, dt);
//        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
 //   }

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
void HungControlTFController::moveAllMotors(HungControlTFModel& subject, double dt) {
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
     
}

// Get actions from evolutionAdapter, transform them to this structure, and apply them
void HungControlTFController::updateActions(HungControlTFModel& subject, double dt) {
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
vector< vector <double> > HungControlTFController::transformActions(vector< vector <double> > actions)
{
	double min=30;
	double max=70;
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

//double HungControlTFController::displacement(HungControlTFModel& subject) {
//    std::vector<double> finalPosition = subject.getBallCOM();

    // 'X' and 'Z' are irrelevant. Both variables measure lateral direction
    //assert(finalPosition[0] > 0); //Negative y-value indicates a flaw in the simulator that run (tensegrity went 'underground')

//    const double newX = finalPosition[0];
//    const double newZ = finalPosition[2];
//    const double oldX = initPosition[0];
//    const double oldZ = initPosition[2];

//    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
//                                      (newZ-oldZ) * (newZ-oldZ));
//    return distanceMoved;
//}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void HungControlTFController::applyActions(HungControlTFModel& subject, vector< vector <double> > act)
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

// So far, only score used for eventual fitness calculation of an Escape Model
// is the maximum distance from the origin reached during that subject's episode
void HungControlTFController::onTeardown(HungControlTFModel& subject) {
    std::vector<double> scores; //scores[0] == displacement, scores[1] == energySpent
    //  double distance = displacement(subject);
    double energySpent = totalEnergySpent(subject);

    //Invariant: For now, scores must be of size 2 (as required by endEpisode())
    // scores.push_back(distance);
//    scores.push_back(energySpent);

    std::cout << "Tearing down" << std::endl;
//    evolutionAdapter.endEpisode(scores);

    // If any of subject's dynamic objects need to be freed, this is the place to do so
}

// Basically the same as Steve's totalEnergySpent function from Escape_T6Controller
// with a few modifications
// 1. It is necessary to turn on bool hist in ""Model.cpp
// 2. This function needs to be declared in header file
// 3. Controller contains function, and Model created in HungControlTFModel is taken as parameter.
// 4. Instead of using getAllMuscles(), we can use find<tgBasicActuator> to determine flexion for Dennis's model
double HungControlTFController::totalEnergySpent(HungControlTFModel& subject) {

    double totalEnergySpent=0;

  // std::vector<tgBasicActuator* > tmpStrings = subject.getAllMuscles();
    std::vector<tgBasicActuator*> tmpFlexion = subject.find<tgBasicActuator>("flexion");
    for(size_t i=0; i<tmpFlexion.size(); i++) {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpFlexion[i]->getHistory();

       for(size_t j=1; j<stringHist.tensionHistory.size(); j++) {
           const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power         
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Vestigial code
                motorSpeed = 0;
            const double workDone = previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    return totalEnergySpent;
    cout << totalEnergySpent << endl;
}

btVector3 HungControlTFController::endEffectorCOM(HungControlTFModel& subject) {
	const std::vector<tgRod*> endEffector = subject.find<tgRod>("endeffector");
	assert(!endEffector.empty());
	return endEffector[0]->centerOfMass();
}

