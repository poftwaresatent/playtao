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
   \file parse.cpp
   \author Roland Philippsen
*/

#include "parse.hpp"
#include <wbc/util/urdf_to_tao.hpp>
#include <wbc/parse/TiXmlBRParser.hpp>
#include <wbc/core/BranchingRepresentation.hpp>


taoNodeRoot * parse_sai_xml_file(char const * filename) throw(std::runtime_error)
{
  wbc::TiXmlBRParser parser;
  wbc::BranchingRepresentation * brep(parser.parse(filename));
  
  // brep never gets deleted... ah well.
  return brep->rootNode();
}


#ifndef HAVE_URDF

taoNodeRoot * parse_urdf_file(char const * filename, std::string const & tao_root_name,
			      urdf_to_tao::LinkFilter const * opt_link_filter,
			      std::vector<std::string> * tao_id_to_link_name_map,
			      std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error)
{
  throw runtime_error("parse_urdf_file(" + string(filename)
		      + "): support for ROS urdf not built in");
}

#else // HAVE_URDF

#include <urdf/model.h>

taoNodeRoot * parse_urdf_file(char const * filename, std::string const & tao_root_name,
			      urdf_to_tao::LinkFilter const * opt_link_filter,
			      std::vector<std::string> * tao_id_to_link_name_map,
			      std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(filename);
  urdf::Model urdf_model;
  if ( ! urdf_model.initXml(&urdf_xml)) {
    throw runtime_error("parse_urdf_file(" + string(filename) + "): urdf::Model::initXml() failed");
  }
  taoNodeRoot * root(0);
  if (opt_link_filter) {
    root = urdf_to_tao::convert(urdf_model,
				tao_root_name,
				*opt_link_filter,
				tao_id_to_link_name_map,
				tao_id_to_joint_name_map);
  }
  else {
    root = urdf_to_tao::convert(urdf_model,
				tao_root_name,
				urdf_to_tao::DefaultLinkFilter(),
				tao_id_to_link_name_map,
				tao_id_to_joint_name_map);
  }
  return root;
}

#endif // HAVE_URDF
