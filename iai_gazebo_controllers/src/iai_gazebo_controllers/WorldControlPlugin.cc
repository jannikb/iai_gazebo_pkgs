/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Andrei Haidu,
 *  Institute for Artificial Intelligence, Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "iai_gazebo_controllers/WorldControlPlugin.hh"

using namespace iai_gazebo_controllers;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldControlPlugin)

//////////////////////////////////////////////////
WorldControlPlugin::WorldControlPlugin()
{
}

//////////////////////////////////////////////////
WorldControlPlugin::~WorldControlPlugin()
{
}

//////////////////////////////////////////////////
void WorldControlPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// set the world ptr
	this->world = _parent;

	// get the delay value
	if (!_sdf->HasElement("startDelay"))
	{
		std::cout << "Missing parameter <startDelay>, default to 0" << std::endl;
		this->startDelay = 0;
	}
	else this->startDelay = _sdf->Get<double>("startDelay");


	// check if the world is paused
	if (_parent->IsPaused())
	{
		std::cout << "World started in pause mode, starting sim in "
				<<  this->startDelay << " sec.." << std::endl;
	}
	else
	{
		//pause the world
		_parent->SetPaused(true);
		std::cout << "Paused the world, starting sim in "
				<<  this->startDelay << " sec.." << std::endl;
	}

    // thread for checking if the starting delay finished
	this->checkStartDelay =
			new boost::thread(&WorldControlPlugin::CheckStartDelayWorker, this);
}

//////////////////////////////////////////////////
void WorldControlPlugin::CheckStartDelayWorker()
{
	while(this->world->IsPaused())
	{
//		std::cout << "Sleeping for "
//				<<  this->startDelay << " sec.." << std::endl;

		// sleep for the given period
		usleep(this->startDelay * 1000000);

//		std::cout << "Unpausing the world after "
//				<<  this->startDelay << " sec.." << std::endl;

		// unpause the world
		this->world->SetPaused(false);
	}
}
