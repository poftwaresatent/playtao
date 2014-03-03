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
   \file udp_robot_api.cpp
   \author Roland Philippsen
*/

#include "udp_robot_api.hpp"
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
  
  
  UDPRobotAPI::
  UDPRobotAPI(uint64_t npos, uint64_t nvel, uint64_t nforce)
    : buf_npos_(0),
      buf_nvel_(0),
      buf_nforce_(0),
      pos_(0),
      vel_(0),
      force_(0),
      buffer_(0),
      initialized_(false)
  {
    //// we used to resize on the fly, but that created confusion.
    initBuffer(npos, nvel, nforce);
  }
  
  
  UDPRobotAPI::
  ~UDPRobotAPI()
  {
    shutdown();
    free(buffer_);
  }
  
  
  jspace::Status UDPRobotAPI::
  readState(jspace::State & state)
  {
    jspace::Status status;
    
    try {
      receiveState();
      
      state.position_.resize(npos_);
      for (uint64_t ii(0); ii < npos_; ++ii) { // float vs double, cannot memcpy()
	state.position_[ii] = pos_[ii];
      }
      
      state.velocity_.resize(nvel_);
      for (uint64_t ii(0); ii < nvel_; ++ii) {
	state.velocity_[ii] = vel_[ii];
      }
      
      state.force_.resize(nforce_);
      for (uint64_t ii(0); ii < nforce_; ++ii) {
	state.force_[ii] = force_[ii];
      }
      
      struct timeval now;
      if (0 != gettimeofday(&now, NULL)) {
	state.time_sec_ = 0;
	state.time_usec_ = 0;
      }
      else {
	state.time_sec_ = now.tv_sec;
	state.time_usec_ = now.tv_usec;
      }
    }
    catch (runtime_error const & ee) {
      status.ok = false;
      status.errstr = ee.what();
    }
    
    return status;
  }
  
  
  jspace::Status UDPRobotAPI::
  writeCommand(jspace::Vector const & command)
  {
    return jspace::Status(false, "calling playtao::UDPRobotAPI::writeCommand() makes no sense");
  }
  
  
  void UDPRobotAPI::
  shutdown()
  {
    if (initialized_) {
      close(udp_sock_fd_);
    }
    initialized_ = false;
  }
  
  
  void UDPRobotAPI::
  purge()
  {
    if (initialized_) {
      char trash[1024];
      while (0 < recv(udp_sock_fd_, &trash, 1024, 0)) { /*nop*/ }
    }
  }
  
  
  void UDPRobotAPI::
  init(std::string const & port,
       int ai_family) throw(std::runtime_error)
  {
    if (initialized_) {
      throw runtime_error("playtao::UDPRobotAPI::init(): already initialized");
    }
    
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
      std::ostringstream msg;
      msg << "playtao::UDPRobotAPI::init(): port `" << port
	  << "': getaddrinfo() failed: " << gai_strerror(gaistatus);
      throw runtime_error(msg.str());
    }
    
    struct addrinfo * rp;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      udp_sock_fd_ = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (udp_sock_fd_ == -1)
	continue;
      if (bind(udp_sock_fd_, rp->ai_addr, rp->ai_addrlen) == 0) {
	std::cout << "DEBUG playtao::UDPRobotAPI::init(): bound to port " << port << "\n";
	break;
      }
      close(udp_sock_fd_);
    }
    freeaddrinfo(result);           /* No longer needed */
    if (rp == NULL) {
      std::ostringstream msg;
      msg << "playtao::UDPRobotAPI::init(): port `" << port
	  << "': bind() failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
    
    purge();
    initialized_ = true;
  }
  
  
  void UDPRobotAPI::
  receiveState() throw(std::runtime_error)
  {
    if ( ! buffer_) {
      throw runtime_error("playtao::UDPRobotAPI::receiveState(): not initialized");
    }
    
    // peek non-blocking at the beginning of the message
    ssize_t nread;
    struct { uint64_t npos, nvel, nforce; } peek;
    nread = recv(udp_sock_fd_, &peek, sizeof(peek), MSG_PEEK | MSG_DONTWAIT);
    if (-1 == nread) {
      if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
	return;
      }
      ostringstream msg;
      msg << "playtao::UDPRobotAPI::receiveState(): peek [recv()] failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
    
    //// we used to resize on the fly, but that created confusion.
    if ((peek.npos != npos_) || (peek.nvel != nvel_) || (peek.nforce != nforce_)) {
      ostringstream msg;
      msg << "playtao::UDPRobotAPI::receiveState(): size mismatch:  npos: want " << npos_ << " got " << peek.npos
	  << "  nvel: want " << nvel_ << " got " << peek.nvel << "  nforce: want " << nforce_ << " got "
	  << peek.nforce;
      throw runtime_error(msg.str());
    }
    
#ifdef DEBUG
    fprintf(stderr, "receiving  npos: %zu  nvel: %zu  nforce: %zu\n",
	    (size_t) peek.npos, (size_t) peek.nvel, (size_t) peek.nforce);
#endif // DEBUG
    
    //// we used to resize on the fly, but that created confusion.
    //     // resize buffer if required, based on the received header data
    //     initBuffer(peek.npos, peek.nvel, peek.nforce);
    
    // read the entire message
    nread = recv(udp_sock_fd_, buffer_, nbytes_, 0);
    if (-1 == nread) {
      ostringstream msg;
      msg << "playtao::UDPRobotAPI::receiveState(): read [recv()] failed: " << strerror(errno);
      throw runtime_error(msg.str());
    }
    if (nread != static_cast<ssize_t>(nbytes_)) {
      ostringstream msg;
      msg << "playtao::UDPRobotAPI::receiveState(): protocol error: expected " << nbytes_
	  << " bytes but got " << nread;
      throw runtime_error(msg.str());
    }

#ifdef DEBUG
    char const * fieldname[] = { "pos  ", "vel  ", "force" };
    float const * field[] = { pos_, vel_, force_ };
    size_t ndata[] = { peek.npos, peek.nvel, peek.nforce };
    for (size_t ii(0); ii < 3; ++ii) {
      fprintf(stderr, "  %s:  ", fieldname[ii]);
      for (size_t jj(0); jj < ndata[ii]; ++jj) {
    	if (isinf(field[ii][jj])) {
	  fprintf(stderr, " inf    ");
	}
	else if (isnan(field[ii][jj])) {
	  fprintf(stderr, " nan    ");
	}
	else if (fabs(fmod(field[ii][jj], 1)) < 1e-6) {
	  fprintf(stderr, "%- 7d  ", static_cast<int>(rint(field[ii][jj])));
	}
	else {
	  fprintf(stderr, "% 6.4f  ", field[ii][jj]);
	}
      }
      fprintf(stderr, "\n");
    }
#endif // DEBUG
  }
  
  
  void UDPRobotAPI::
  initBuffer(uint64_t npos, uint64_t nvel, uint64_t nforce)
    throw(std::runtime_error)
  {
    if (buffer_ && (npos == npos_) && (nvel == nvel_) && (nforce == nforce_)) {
      return;
    }
    
    npos_ = npos;
    nvel_ = nvel;
    nforce_ = nforce;
    nbytes_ = 3 * sizeof(uint64_t) + (npos + nvel + nforce) * sizeof(float);
    
    void * nb(realloc(buffer_, nbytes_));
    if (0 == nb) {
      free(buffer_);
      buffer_ = 0;
    }
    else {
      buffer_ = (char*) nb;
    }
    if ( ! buffer_) {
      throw runtime_error("playtao::UDPRobotAPI::initBuffer(): out of memory");
    }
    
    memset(buffer_, 0, nbytes_);
    
    buf_npos_ = (uint64_t *) buffer_;
    buf_nvel_ = buf_npos_ + 1;
    buf_nforce_ = buf_nvel_ + 1;
    pos_ = (float*) (buf_nforce_ + 1);
    vel_ = pos_ + npos_;
    force_ = vel_ + nvel_;
  }

}
