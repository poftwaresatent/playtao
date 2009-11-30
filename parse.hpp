/*
 * Play TAO --- a tool for the Stanford Whole-Body Control Framework
 *              http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2009 Stanford University. All rights reserved.
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
   \file parse.hpp
   \author Roland Philippsen
*/

#ifndef PLAYTAO_PARSE_HPP
#define PLAYTAO_PARSE_HPP

#include <stdexcept>
#include <vector>
#include <string>

namespace urdf_to_tao {
  class LinkFilter;
}

class taoNodeRoot;

taoNodeRoot * parse_sai_xml_file(char const * filename) throw(std::runtime_error);

taoNodeRoot * parse_urdf_file(char const * filename, std::string const & tao_root_name,
			      urdf_to_tao::LinkFilter const * opt_link_filter,
			      std::vector<std::string> * tao_id_to_link_name_map,
			      std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error);

#endif // PLAYTAO_PARSE_HPP
