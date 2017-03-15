/*
 * Copyright © 2012, United States Government, as represented by the
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

#ifndef HipCONTROL_TF_MODEL_H
#define HipCONTROL_TF_MODEL_H

/**
 * @file HipControlTFModel.h
 * @brief Defines Knee Flexion Model 
 * @author Dennis Castro
 * @version 6.2.0
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
#include "core/tgRod.h"
// The C++ Standard Library
#include <vector>
#include <map>
#include <set>
#include <string>

// Forward declarations
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * A class that constructs a Knee model consisting of two sections; 
 * The lower leg and the upper leg   portions. This iteration avoids using a controller and instead
 * uses the new (to v1.1) ability to define pretension in a
 * tgBasicActuator's constructor, but is a working progress to use a controller
 */
class HipControlTFModel : public tgSubject<HipControlTFModel>, public tgModel
{
public:

    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in.
     */
    HipControlTFModel();

    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~HipControlTFModel();

    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);

    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();

    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);

    /**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself
     */
    virtual void onVisit(tgModelVisitor& r);

    /**
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgBasicActuator*>& getAllMuscles() const;

private:

    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     * @param[in] height: the total height of the human
     * @param[in] femur: the proportion constant of the femur to height
     * @param[in] tibia: the proportion constant of the tibia to height
     */
    static void addNodes(tgStructure& s,
                            double height,
                            double femur,
                            double tibia);

   /* static void addNodesB(tgStructure& tetra,
                            double edge,
                            double width,
                            double height);
*/
    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
   /* static void addPairsB(tgStructure& tetra);
*/

    /**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addMuscles(tgStructure& s);

private:
    /**
     * A list of all of the basic actuators. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgBasicActuator*> allMuscles;
    std::vector<btVector3> nodePositions;
};

#endif  // HipCONTROL_KNEE_MODEL_H