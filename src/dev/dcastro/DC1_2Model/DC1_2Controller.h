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

#ifndef DC1_2_CONTROLLER_H
#define DC1_2_CONTROLLER_H

/**
 * @file DC1_2Controller.h
 * @brief Contains the definition of class ControlTFController
 * @author Dennis Castro
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h" //************What is this?***********8
#include <vector>

// Forward declarations
class DC1_2Model;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for DC1_2Model. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class DC1_2Controller : public tgObserver<DC1_2Model>
{
public:
	
  /**
   * Construct a DC1_2Controller with the initial preferred length.
   *
   */
  
  // Note that currently this is calibrated for decimeters.
	DC1_2Controller(const double prefLength, double timestep);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~DC1_2Controller() { }

  virtual void onSetup(DC1_2Model& subject);
    
  virtual void onStep(DC1_2Model& subject, double dt);

protected:

  virtual vector< vector <double> > transformActions(vector< vector <double> > act);

  virtual void applyActions (DC1_2Model& subject, vector< vector <double> > act);

private:
  double m_initialLengths;
  double m_totalTime;
  double dt;
  //AnnealAdapter evolutionAdapter;
    
  void setFlexionTargetLength(DC1_2Model& subject, double dt);
 // void setAnconeusTargetLength(ScarrArmModel& subject, double dt);
  void moveAllMotors(DC1_2Model& subject, double dt);
  void updateActions(DC1_2Model& subject, double dt);
};

#endif // DC1_2_CONTROLLER_H
