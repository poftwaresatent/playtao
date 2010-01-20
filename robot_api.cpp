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
#include <fstream>
#include <sstream>
#include <sys/time.h>


namespace {
  
  class RobotAPI
    : public wbc::RobotAPI
  {
  public:
    RobotAPI(std::string const & filename)
      : m_filename(filename),
	m_fstream(filename.c_str())
    {}
    
    
    // probably redundant...
    ~RobotAPI() { m_fstream.close(); }
    
    
    virtual bool readSensors(SAIVector & jointAngles,
			     SAIVector & jointVelocities,
			     timeval & acquisition_time,
			     SAIMatrix * opt_forces)
    {
      static int const buflen(1024);
      static char buffer[buflen];
      if ( ! m_fstream.getline(buffer, buflen)) {
	cerr << "EOF or invalid file \"" << m_filename << "\"\n";
	return false;
      }
      istringstream line(buffer);
      
      for ( int ii(0); ii < jointAngles.size(); ++ii ) {
	line >> jointAngles[ii];
	if ( ! line) {
	  break;
	}
      }
      if ( ! line) {
	cerr << "incomplete line \"" << buffer << "\" in file \"" << m_filename << "\"\n";
	return false;
      }
      
      jointVelocities.zero();
      if (opt_forces) {
	opt_forces->setSize(6, jointAngles.size(), true);
      }
      if (0 != gettimeofday(&acquisition_time, NULL)) {
	cerr << "gettimeofday() failed\n";
	return false;
      }
      
      return true;
    }
    
    
    virtual bool writeCommand(SAIVector const & command) { return true; }
    
    
    virtual void shutdown() const {}
    
    
  protected:
    std::string const m_filename;
    std::ifstream m_fstream;
  };
  
}


wbc::RobotAPI * create_robot(std::string const & spec)
{
  return new RobotAPI(spec);
}
