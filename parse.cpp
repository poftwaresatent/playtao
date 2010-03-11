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
#include <wbc/ros/urdf_to_tao.hpp>
#include <wbc/parse/TiXmlBRParser.hpp>
#include <wbc/core/BranchingRepresentation.hpp>
#include <wbc/ros/Model.hpp>
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
			      wbcros::LinkFilter const * opt_link_filter,
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
  URDFTAOContainer(wbc::tao_tree_info_s * tree_)
    : tree(tree_) { }
  
  virtual ~URDFTAOContainer() {
    delete tree;
  }
  
  virtual taoNodeRoot * getRoot() {
    return tree->root;
  }
  
  wbc::tao_tree_info_s * tree;
};


TAOContainer * parse_urdf_file(char const * filename, std::string const & tao_root_name,
			      wbcros::LinkFilter const * opt_link_filter,
			      std::vector<std::string> * tao_id_to_link_name_map,
			      std::vector<std::string> * tao_id_to_joint_name_map) throw(std::runtime_error)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(filename);
  urdf::Model urdf_model;
  if ( ! urdf_model.initXml(&urdf_xml)) {
    throw runtime_error("parse_urdf_file(" + string(filename) + "): urdf::Model::initXml() failed");
  }
  wbc::tao_tree_info_s * tree;
  if (opt_link_filter) {
    tree = wbcros::convert_urdf_to_tao(urdf_model, tao_root_name, *opt_link_filter);
  }
  else {
    tree = wbcros::convert_urdf_to_tao(urdf_model, tao_root_name, wbcros::DefaultLinkFilter());
  }
  
  if (tao_id_to_link_name_map) {
    size_t const nn(tree->info.size());
    tao_id_to_link_name_map->resize(nn);
    for (size_t ii(0); ii < nn; ++ii) {
      (*tao_id_to_link_name_map)[ii] = tree->info[ii].link_name;
    }
  }
  
  if (tao_id_to_joint_name_map) {
    size_t const nn(tree->info.size());
    tao_id_to_joint_name_map->resize(nn);
    for (size_t ii(0); ii < nn; ++ii) {
      (*tao_id_to_joint_name_map)[ii] = tree->info[ii].joint_name;
    }
  }
  
  return new URDFTAOContainer(tree);
}

#endif // HAVE_URDF


#ifndef HAVE_ROS

TAOContainer * parse_ros_parameter(std::string const & node_name,
				   std::string const & urdf_param_name) throw(std::runtime_error)
{
  throw runtime_error("parse_ros_parameter() only available when ROS support is built into WBC");
}

#else // HAVE_ROS


class PR2TAOContainer: public TAOContainer {
public:
  PR2TAOContainer(std::string const & param_prefix)
    : node(0),
      model(param_prefix)
  {
    int argc(1);
    char * argv[1];
    argv[0] = strdup("PR2TAOContainer");
    ros::init(argc, argv, "PR2TAOContainer", ros::init_options::NoSigintHandler);
    free(argv[0]);
  }
  
  virtual ~PR2TAOContainer()
  {
    delete node;
  }
  
  void init(std::string const & urdf_param_name) throw(std::runtime_error)
  {
    node = new ros::NodeHandle;//g("~");
    model.initFromParam(*node, urdf_param_name, 0, 1);
  }
  
  virtual taoNodeRoot * getRoot() {
    return model.tao_trees_[0]->root;
  }
  
  ros::NodeHandle * node;
  wbcros::Model model;
};


TAOContainer * parse_ros_parameter(std::string const & param_prefix,
				   std::string const & urdf_param_name) throw(std::runtime_error)
{
  PR2TAOContainer * container(new PR2TAOContainer(param_prefix));
  container->init(urdf_param_name);
  return container;
}

#endif // HAVE_ROS
