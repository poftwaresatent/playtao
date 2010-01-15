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
#include <tao/dynamics/taoNode.h>


class SAITAOContainer: public TAOContainer {
public:
  SAITAOContainer(wbc::BranchingRepresentation * brep_)
    : brep(brep_) { }
  
  virtual ~SAITAOContainer() {
    delete brep->rootNode(); // Yes, that's right, nobody felt it was necessary to delete this dude in WBC.
    delete brep;
  }
  
  virtual taoNodeRoot * getRoot() {
    return brep->rootNode();
  }
  
  wbc::BranchingRepresentation * brep;
};



TAOContainer * parse_sai_xml_file(char const * filename) throw(std::runtime_error)
{
  wbc::TiXmlBRParser parser;
  return new SAITAOContainer(parser.parse(filename));
}


#ifndef HAVE_URDF

TAOContainer * parse_urdf_file(char const * filename, std::string const & tao_root_name,
			      urdf_to_tao::LinkFilter const * opt_link_filter,
			      std::vector<std::string> * tao_id_to_link_name_map,
			      std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error)
{
  throw runtime_error("parse_urdf_file(" + string(filename)
		      + "): support for ROS urdf not built in");
  return 0;
}

#else // HAVE_URDF

#include <urdf/model.h>


class URDFTAOContainer: public TAOContainer {
public:
  URDFTAOContainer(taoNodeRoot * root_)
    : root(root_) { }
  
  virtual ~URDFTAOContainer() {
    delete root;
  }
  
  virtual taoNodeRoot * getRoot() {
    return root;
  }
  
  taoNodeRoot * root;
};


TAOContainer * parse_urdf_file(char const * filename, std::string const & tao_root_name,
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
  if (opt_link_filter) {
    return new URDFTAOContainer(urdf_to_tao::convert(urdf_model,
						     tao_root_name,
						     *opt_link_filter,
						     tao_id_to_link_name_map,
						     tao_id_to_joint_name_map));
  }
  return new URDFTAOContainer(urdf_to_tao::convert(urdf_model,
						   tao_root_name,
						   urdf_to_tao::DefaultLinkFilter(),
						   tao_id_to_link_name_map,
						   tao_id_to_joint_name_map));
}

#endif // HAVE_URDF
