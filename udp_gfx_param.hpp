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
   \file udp_gfx_param.hpp
   \author Roland Philippsen
*/

#ifndef PLAYTAO_UDP_GFX_PARAM_HPP
#define PLAYTAO_UDP_GFX_PARAM_HPP

#include <jspace/RobotAPI.hpp>
#include <stdexcept>

extern "C" {
#include <stdint.h>
#include <sys/socket.h>
}


namespace playtao {
  
  
  struct gfx_param_s {
    gfx_param_s()
      : long_cone_length(1.0),
	long_cone_base(0.05),
	short_cone_length(0.4),
	short_cone_base(0.1),
	link_radius(0.05),
	com_radius(0.1) {}
    
    float long_cone_length;
    float long_cone_base;
    float short_cone_length;
    float short_cone_base;
    float link_radius;
    float com_radius;
  };
  
  
  class UDPGfxParam
  {
  public:
    UDPGfxParam();
    ~UDPGfxParam();
    
    void init(/** port specification, will get passed to
		  getaddrinfo() */
	      std::string const & port,
	      /** use AF_UNSPEC for allowing IPv4 or IPv6, AF_INET for
		  IPv4, or AF_INET6 for IPv6 */
	      int ai_family = AF_UNSPEC) throw(std::runtime_error);
    
    void tryReceive(gfx_param_s & gfx_param) throw(std::runtime_error);
    
  protected:    
    std::string port_;
    int udp_sock_fd_;
    float * buffer_;
  };
  
}

#endif // PLAYTAO_UDP_GFX_PARAM_HPP
