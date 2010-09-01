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
   \file udp_robot_api.hpp
   \author Roland Philippsen
*/

#ifndef PLAYTAO_UDP_ROBOT_API_HPP
#define PLAYTAO_UDP_ROBOT_API_HPP

#include <jspace/RobotAPI.hpp>
#include <stdexcept>

extern "C" {
#include <stdint.h>
#include <sys/socket.h>
}


namespace playtao {
  
  class UDPRobotAPI
    : public jspace::RobotAPI
  {
  public:
    /** We used to allow the ndof to change in each message, but that
	creates more confusion and runtime bugs than are worth that
	added convenience. So now you have to specify the sizes at
	construction time. */
    UDPRobotAPI(uint64_t npos, uint64_t nvel, uint64_t nforce);
    virtual ~UDPRobotAPI();
    
    virtual jspace::Status readState(jspace::State & state);
    virtual jspace::Status writeCommand(jspace::Vector const & command);
    virtual void shutdown();
    
    void purge();
    
    void init(/** port specification, will get passed to
		  getaddrinfo() */
	      std::string const & port,
	      /** use AF_UNSPEC for allowing IPv4 or IPv6, AF_INET for
		  IPv4, or AF_INET6 for IPv6 */
	      int ai_family = AF_UNSPEC) throw(std::runtime_error);
    
  protected:
    void receiveState() throw(std::runtime_error);
    void initBuffer(uint64_t npos, uint64_t nvel, uint64_t nforce) throw(std::runtime_error);

    std::string port_;
    uint64_t npos_, nvel_, nforce_, nbytes_;
    int udp_sock_fd_;
    
    uint64_t * buf_npos_, * buf_nvel_, * buf_nforce_;
    float * pos_, * vel_, * force_;
    char * buffer_;
    
    bool initialized_;
  };
  
}

#endif // PLAYTAO_UDP_ROBOT_API_HPP
