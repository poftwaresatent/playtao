/*
 * Play TAO --- a tool for the Stanford Whole-Body Control Framework
 *              http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file robot_api.cpp
   \author Roland Philippsen
*/

#include "robot_api.hpp"
#include <saimatrix/SAIVector.h>
#include <saimatrix/SAIMatrix.h>
#include <sys/time.h>


namespace {
  
  class RobotAPI
    : public wbc::RobotAPI
  {
  public:
    virtual bool readSensors(SAIVector & jointAngles,
			     SAIVector & jointVelocities,
			     timeval & acquisition_time,
			     SAIMatrix * opt_forces)
    {
      for ( int ii(0); ii < jointAngles.size(); ++ii )
	jointAngles[ii] = 0.1 + 0.001 * ii;
      for ( int ii(0); ii < jointVelocities.size(); ++ii )
	jointVelocities[ii] = -0.1 - 0.001 * ii;
      if (opt_forces) {
	opt_forces->setSize(6, jointAngles.size(), true);
	for (int icol(0); icol < opt_forces->column(); ++icol)
	  for (int irow(0); irow < opt_forces->row(); ++irow)
	    opt_forces->elementAt(irow, icol) = 5 * icol + 0.2 * irow;
      }
      if (0 != gettimeofday(&acquisition_time, NULL)) {
	cerr << "gettimeofday() failed\n";
	return false;
      }
      return true;
    }
    
    virtual bool writeCommand(SAIVector const & command) { return true; }
    
    virtual void shutdown() const {}
  };
  
}


wbc::RobotAPI * create_robot(std::string const & spec)
{
  return new RobotAPI();
}
