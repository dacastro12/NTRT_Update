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

/**
 * @file tgLinearStringInfo.cpp
 * @brief Implementation of class tgLinearStringInfo
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

#include "tgLinearStringInfo.h"

#include "core/Muscle2P.h"

tgLinearStringInfo::tgLinearStringInfo(const tgLinearString::Config& config) : 
m_config(config),
tgConnectorInfo()
{}

tgLinearStringInfo::tgLinearStringInfo(const tgLinearString::Config& config, tgTags tags) : 
m_config(config),
tgConnectorInfo(tags)
{}

tgLinearStringInfo::tgLinearStringInfo(const tgLinearString::Config& config, const tgPair& pair) :
m_config(config),
tgConnectorInfo(pair)
{}
    

tgConnectorInfo* tgLinearStringInfo::createConnectorInfo(const tgPair& pair)
{
    return new tgLinearStringInfo(m_config, pair);
}

void tgLinearStringInfo::initConnector(tgWorld& world)
{
	///@todo confirm m_muscle2P isn't occupied. See example in Corde
	
    // Note: Muscle2P holds pointers to things in the world, but it doesn't actually have any in-world representation.
    m_muscle2P = createMuscle2P();
}

tgModel* tgLinearStringInfo::createModel(tgWorld& world)
{
    // Don't have to do anything in the world for a Muscle2P...
    // @todo: set the name based on joined tags, or remove name from the model...
    //std::cout << "tgLinearStringInfo::createModel" << std::endl;
    
    // ensure connector has been initialized
    assert(m_muscle2P);
    return new tgLinearString(m_muscle2P, getTags(), m_config);
}

double tgLinearStringInfo::getMass() 
{
    // @todo: calculate a mass? Muscle2P doesn't have physics...
    return 0;
}


Muscle2P* tgLinearStringInfo::createMuscle2P()
{
    //std::cout << "tgLinearStringInfo::createMuscle2P()" << std::endl;
    
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo() << std::endl;
    //std::cout << "  getFromRigidInfo(): " << getFromRigidInfo()->getRigidInfoGroup() << std::endl;
    
    // @todo: need to check somewhere that the rigid bodies have been set...
    btRigidBody* fromBody = getFromRigidBody();
    btRigidBody* toBody = getToRigidBody();

    btVector3 from = getFromRigidInfo()->getConnectionPoint(getFrom(), getTo(), m_config.rotation);
    btVector3 to = getToRigidInfo()->getConnectionPoint(getTo(), getFrom(), m_config.rotation);

    return new Muscle2P(fromBody, from, toBody, to, m_config.stiffness, m_config.damping);
}
    
