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
   \file udp_gfx_param.cpp
   \author Roland Philippsen
*/

#include "udp_gfx_param.hpp"
#include <sstream>
#include <iostream>

extern "C" {
#include <sys/time.h>
#include <netdb.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
}

//#undef DEBUG
#ifdef DEBUG
# include <stdio.h>
#endif // DEBUG

using namespace std;


namespace playtao {
  
  
  UDPGfxParam::
  UDPGfxParam()
    : buffer_(0)
  {
  }
  
  
  UDPGfxParam::
  ~UDPGfxParam()
  {
    if (buffer_) {
      close(udp_sock_fd_);
      delete[] buffer_;
    }
  }
  
  
  void UDPGfxParam::
  init(std::string const & port,
       int ai_family) throw(std::runtime_error)
  {
    if (buffer_) {
      throw runtime_error("playtao::UDPGfxParam::init(): already initialized");
    }
    
    buffer_ = (float*) malloc(sizeof(gfx_param_s));
    if (0 == buffer_) {
      throw runtime_error("playtao::UDPGfxParam::initBuffer(): out of memory");
    }
    memset(buffer_, 0, sizeof(gfx_param_s));
    
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = ai_family;
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
    hints.ai_protocol = 0;          /* Any protocol */
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    
    struct addrinfo * result;
    int const gaistatus(getaddrinfo(NULL, port.c_str(), &hints, &result));
    if (gaistatus != 0) {
      free(buffer_);
      buffer_ = 0;
      std::ostringstream msg;
      msg << "playtao::UDPGfxParam::init(): port `" << port
	  << "': getaddrinfo() failed: " << gai_strerror(gaistatus);
      throw runtime_error(msg.str());
    }
    
    struct addrinfo * rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      udp_sock_fd_ = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (udp_sock_fd_ == -1)
	continue;
      if (bind(udp_sock_fd_, rp->ai_addr, rp->ai_addrlen) == 0) {
	std::cout << "DEBUG playtao::UDPGfxParam::init(): bound to port " << port << "\n";
	break;
      }
      close(udp_sock_fd_);
    }
    freeaddrinfo(result);           /* No longer needed */
    if (rp == NULL) {
      free(buffer_);
      buffer_ = 0;
      std::ostringstream msg;
      msg << "playtao::UDPGfxParam::init(): port `" << port
	  << "': bind() failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
  }
  
  
  bool UDPGfxParam::
  tryReceive(gfx_param_s & gfx_param) throw(std::runtime_error)
  {
    if ( ! buffer_) {
      throw runtime_error("playtao::UDPGfxParam::tryReceive(): not initialized");
    }
    
    // peek one byte
    ssize_t nread;
    uint8_t peek;
    nread = recv(udp_sock_fd_, &peek, 1, MSG_PEEK | MSG_DONTWAIT);
    if (-1 == nread) {
      if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
	return false;
      }
      ostringstream msg;
      msg << "playtao::UDPGfxParam::tryReceive(): peek [recv()] failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
    
    // read the entire message
    nread = recv(udp_sock_fd_, buffer_, sizeof(gfx_param_s), 0);
    if (-1 == nread) {
      ostringstream msg;
      msg << "playtao::UDPGfxParam::tryReceive(): read [recv()] failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
    if (nread != sizeof(gfx_param_s)) {
      ostringstream msg;
      msg << "playtao::UDPGfxParam::tryReceive(): protocol error: expected " << sizeof(gfx_param_s)
	  << " bytes but got " << nread;
      throw runtime_error(msg.str());
    }
    
    gfx_param.long_cone_length = buffer_[0];
    gfx_param.long_cone_base = buffer_[1];
    gfx_param.short_cone_length = buffer_[2];
    gfx_param.short_cone_base = buffer_[3];
    gfx_param.link_radius = buffer_[4];
    gfx_param.com_radius = buffer_[5];
    gfx_param.joint_radius = buffer_[6];
    gfx_param.joint_length = buffer_[7];
    
    return true;
  }
  
}
