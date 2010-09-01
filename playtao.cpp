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
   \file playtao.cpp
   \author Roland Philippsen
*/

#include "udp_robot_api.hpp"
#include "udp_gfx_param.hpp"

#include <jspace/Model.hpp>
#include <jspace/RobotAPI.hpp>
#include <jspace/tao_dump.hpp>
#include <jspace/test/sai_brep.hpp>
#include <jspace/test/sai_brep_parser.hpp>

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoVar.h>
#include <tao/utility/TaoDeMassProp.h>

#include <gfx/wrap_glut.hpp>
#include <gfx/viewport.hpp>
#include <gfx/gltrackball.h>

#include <boost/shared_ptr.hpp>

#include <stdexcept>

#include <fstream>
#include <err.h>
#include <signal.h>
#include <cstring>
#include <sys/time.h>


using namespace gfx;
using namespace boost;
using namespace std;


static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<jspace::RobotAPI> robot_api;
static boost::shared_ptr<jspace::State> jspace_state;
static GLUquadricObj * qobj(0);
static playtao::UDPGfxParam udp_gfx_param;
static playtao::gfx_param_s gfx_param;

static size_t tick(0);
static trackball_state * trackball;
static int left_button_down(0);
static int winwidth(400);
static int winheight(400);
static gfx::Viewport vp_tao(     0, 0, 1, 1);//0.5, 1);
//static gfx::Viewport vp_jspace(0.5, 0,   1, 1);

static int verbosity(0);
static string sai_filename("");
static int n_iterations(-1);	// -1 means graphics mode, user presses 'q' to quit
static unsigned int gfx_timer_ms(20);
static std::string udp_robot_port("3589");
static std::string udp_gfx_port("2763");

static void usage(ostream & os);
static void parse_options(int argc, char ** argv);
static void init_glut(int * argc, char ** argv, int width, int height);
static bool update();
static void reshape(int width, int height);
static void draw();
static void keyboard(unsigned char key, int x, int y);
static void timer(int handle);
static void mouse(int button, int state, int x, int y);
static void motion(int x, int y);
static void cleanup(void);
static void handle(int signum);
static jspace::Model * load_model() throw(std::runtime_error);


int main(int argc, char ** argv)
{
  if (atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  if (signal(SIGINT, handle) == SIG_ERR) 
    err(EXIT_FAILURE, "signal(SIGINT)");
  if (signal(SIGHUP, handle) == SIG_ERR) 
    err(EXIT_FAILURE, "signal(SIGHUP)");
  if (signal(SIGTERM, handle) == SIG_ERR) 
    err(EXIT_FAILURE, "signal(SIGTERM)");
  
  parse_options(argc, argv);
  //  vp_jspace.MimicBounds(&vp_tao);
  
  try {
    udp_gfx_param.init(udp_gfx_port);
    gfx_param.long_cone_length = 0.5;
    gfx_param.long_cone_base = 0.02;
    gfx_param.short_cone_length = 0.2;
    gfx_param.short_cone_base = 0.02;
    gfx_param.link_radius = 0.03;
    gfx_param.com_radius = 0.02;
    gfx_param.joint_radius = 0.04;
    gfx_param.joint_length = 0.09;
    
    model.reset(load_model());
    
    size_t const ndof(model->getNDOF());
    playtao::UDPRobotAPI * robot(new playtao::UDPRobotAPI(ndof, ndof, ndof));
    robot->init(udp_robot_port);
    robot_api.reset(robot);
    
    if (n_iterations < 0) {
      trackball = gltrackball_init();
      qobj = gluNewQuadric();
      gluQuadricNormals(qobj, GLU_SMOOTH);
      init_glut(& argc, argv, winwidth, winheight);
      glutMainLoop();
    }
    else {
      for (int ii(0); ii < n_iterations; ++ii) {
	if ( ! update()) {
	  break;
	}
      }
    }
    
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  return 0;
}


jspace::Model * load_model() throw(std::runtime_error)
{
  if (sai_filename.empty()) {
    throw runtime_error("no SAI filename specified");
  }
  
  cout << "loading SAI file " << sai_filename << "\n";
  jspace::test::BRParser brp;
  jspace::test::BranchingRepresentation * brep(brp.parse(sai_filename));
  jspace::tao_tree_info_s * kgm_tree(brep->createTreeInfo());
  cout << "TAO tree:\n";
  jspace::dump_tao_tree_info(cout, kgm_tree, "  ", false);
  delete brep;
  brep = brp.parse(sai_filename); // parse it again to create a second copy for Coriolis / centrifugal
  jspace::tao_tree_info_s * cc_tree(brep->createTreeInfo());
  delete brep;
  
  jspace::Model * mm(new jspace::Model());
  if (0 != mm->init(kgm_tree, cc_tree, &cerr)) {
    delete mm;
    throw runtime_error("jspace::Model::init() failed");
  }
  return mm;
}


void init_glut(int * argc, char ** argv,
	       int width, int height)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
  // // Arghl!!! Contrary to doc, this needs to be done at each
  // // iteration, not just once during init.
  // glEnable(GL_DEPTH_TEST);
  
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);
  
  int handle(glutCreateWindow("playtao"));
  if (0 == handle)
    errx(EXIT_FAILURE, "glutCreateWindow() failed");
  
  {
    GLfloat l_white[] = { 1.0, 1.0, 1.0, 1.0 };
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, l_white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, l_white);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
  }
  
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  
  glutDisplayFunc(draw);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutTimerFunc(gfx_timer_ms, timer, handle);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
}


void reshape(int width, int height)
{
  vp_tao.UpdateShape(width, height);
  //  vp_jspace.UpdateShape(width, height);
  winwidth = width;
  winheight = height;
}


void keyboard(unsigned char key, int x, int y)
{
  switch(key){
  case 'r':
    try {
      jspace::Model * nm(load_model());
      model.reset(nm);
      vp_tao.ResetBounds();
      //      vp_jspace.ResetBounds();
    }
    catch (std::exception const & ee) {
      cout << "oops while trying to reload file: " << ee.what() << "\n";
    }
    break;
  case 'q':
    exit(EXIT_SUCCESS);
    break;
  }
}


/** \note \c pos needs to point to a sufficiently large array, this
    function will happily read over its end otherwise. */
void set_node_state(taoDNode * node, double const * & pos)
{
  for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
    // Well, at first it seems weird that taoJoint::setQ() etc expect
    // a pointer, but this actually makes sense because a joint can
    // have more than one degree of freedom, and the pointer semantics
    // allow us to pass in an array.
    joint->setQ(pos);
    joint->zeroDQ();
    joint->zeroDDQ();
    joint->zeroTau();
    pos += joint->getDOF();
  }
  for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
    set_node_state(child, pos);
  }
}


/** \note \c pos and \c vel need to point to sufficiently large
    arrays, this function will happily write over their ends
    otherwise. */
void get_node_state(taoDNode * node, double * pos, double * vel)
{
  for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
    joint->getQ(pos);
    joint->getDQ(vel);
    int const jdof(joint->getDOF());
    pos += jdof;
    vel += jdof;
  }
  for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
    get_node_state(child, pos, vel);
  }
}


/** \note \c tau needs to point to a sufficiently large array, this
    function will happily read over its end otherwise. */
void add_node_command(taoDNode * node, double const * tau)
{
  for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
    int const jdof(joint->getDOF());
    std::vector<double> foo(jdof);
    joint->getTau(&foo[0]);
    for (int ii(0); ii < jdof; ++ii) {
      foo[ii] += *tau;
      ++tau;
    }
    joint->setTau(&foo[0]);
  }
  for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
    add_node_command(child, tau);
  }
}


//// maybe resurrect here or in jspace/tao_dump.hpp
//
// static void dump_global_frames(std::ostream & os, taoDNode * node, std::string const & prefix)
// {
//   os << prefix << "ID " << node->getID() << ": " << *node->frameGlobal() << "\n";
//   for (taoDNode * child(node->getDChild()); child != 0; child = child->getDSibling()) {
//     dump_global_frames(os, child, prefix);
//   }
// }


bool update()
{
  try {
    udp_gfx_param.tryReceive(gfx_param);
  }
  catch (std::runtime_error const & ee) {
    cerr << "update(): udp_gfx_param.tryReceive() failed: " << ee.what() << "\n";
    return false;
  }
  
  size_t const ndof(model->getNDOF());
  if ( ! jspace_state) {
    jspace_state.reset(new jspace::State());
  }
  
  if ( ! robot_api) {
    jspace_state->init(ndof, ndof, ndof);
  }
  else {
    jspace::Status status(robot_api->readState(*jspace_state));
    if ( ! status) {
      cerr << "update(): robot_api->readState() failed: " << status.errstr << "\n";
      return false;
    }
    if ((jspace_state->position_.size() != static_cast<ssize_t>(ndof))
	|| (jspace_state->velocity_.size() != static_cast<ssize_t>(ndof))
	|| (jspace_state->force_.size() != static_cast<ssize_t>(ndof))) {
      if (verbosity >= 1) {
	cerr << "update(): WARNING state has " << jspace_state->position_.size()
	     << " positions, " << jspace_state->velocity_.size()
	     << " velocities, and " << jspace_state->force_.size()
	     << " forces, but it should have " << ndof << " of each\n";
      }
      jspace_state->resizeAndPadWithZeros(ndof, ndof, ndof);
    }
  }
  
  model->update(*jspace_state);
  
  if (verbosity >= 2) {
    jspace::dump_tao_tree_info(cout, model->_getKGMTree(), "", true);
  }
  
  ++tick;
  return true;
}


void timer(int handle)
{
  if ( ! update()) {
    errx(EXIT_FAILURE, "timer(): update() failed");
  }
  
  glutSetWindow(handle);
  glutPostRedisplay();
  
  glutTimerFunc(gfx_timer_ms, timer, handle);
}


void mouse(int button, int state, int x, int y)
{
  if (GLUT_LEFT_BUTTON == button) {
    if (GLUT_DOWN == state) {
      left_button_down = 1;
      gltrackball_start (trackball, x, y, winwidth, winheight);
    }
    else
      left_button_down = 0;
  }
}


void motion(int x, int y)
{
  if (0 != left_button_down)
    gltrackball_track (trackball, x, y, winwidth, winheight);
}


void cleanup(void)
{
  // if (transform_file) {
  //   // flushing and closing is probably redundant...
  //   (*transform_file) << flush;
  //   transform_file->close();
  //   delete transform_file;
  // }
  if (0 != trackball) {
    free(trackball);
  }
  if (0 != qobj) {
    gluDeleteQuadric(qobj);
  }
}


static bool draw_pipe(Eigen::Vector3d const & p0, Eigen::Vector3d const & p1,
		      double r0, double r1, bool capped = true)
{
  static Eigen::Vector3d const ez(0, 0, 1);
  Eigen::Vector3d ezd(p1 - p0);
  double const ezdnorm(ezd.norm());
  if (ezdnorm < 1e-6) {
    return false;
  }
  
  ezd /= ezdnorm;
  Eigen::Vector3d exd(ezd.cross(ez));
  double const exdnorm(exd.norm());
  if (exdnorm < 1e-6) {
    exd << 1, 0, 0;
  }
  else {
    exd /= exdnorm;
  }
  Eigen::Vector3d eyd(ezd.cross(exd));
  Eigen::Matrix4d transform(Eigen::Matrix4d::Zero());
  transform.block(0, 0, 3, 1) = exd;
  transform.block(0, 1, 3, 1) = eyd;
  transform.block(0, 2, 3, 1) = ezd;
  transform.block(0, 3, 3, 1) = p0;
  transform.coeffRef(3, 3) = 1;
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(transform.data());
  gluCylinder(qobj, r0, r1, ezdnorm, 24, 24);
  if (capped) {
    glPushMatrix();
    glTranslated(0, 0, ezdnorm);
    gluDisk(qobj, 0, r1, 24, 2);
    glPopMatrix();
    glPushMatrix();
    glRotated(180, 1, 0, 0);
    gluDisk(qobj, 0, r0, 24, 2);
    glPopMatrix();
  }
  glPopMatrix();
  
  return true;
}


static void draw_tree(taoDNode /*const*/ * node)
{
  static size_t prev_tick(812379);
  
  Eigen::Vector3d const p0(node->frameGlobal()->translation()[0],
			   node->frameGlobal()->translation()[1],
			   node->frameGlobal()->translation()[2]);
  vp_tao.UpdateBounds(p0[0], p0[1], p0[2]);
  if (true) {
    deFrame foo;
    foo.translation()[0] = gfx_param.short_cone_length;
    foo.translation()[1] = 0;
    foo.translation()[2] = 0;
    deFrame px;
    px.multiply(*node->frameGlobal(), foo);
    foo.translation()[0] = 0;
    foo.translation()[1] = gfx_param.short_cone_length;
    foo.translation()[2] = 0;
    deFrame py;
    py.multiply(*node->frameGlobal(), foo);
    foo.translation()[0] = 0;
    foo.translation()[1] = 0;
    foo.translation()[2] = gfx_param.short_cone_length;
    deFrame pz;
    pz.multiply(*node->frameGlobal(), foo);
    glColor3d(1, 0, 0);
    draw_pipe(p0, Eigen::Vector3d(px.translation()[0],
				  px.translation()[1],
				  px.translation()[2]),
	      gfx_param.short_cone_base, 0);
    glColor3d(0, 1, 0);
    draw_pipe(p0, Eigen::Vector3d(py.translation()[0],
				  py.translation()[1],
				  py.translation()[2]),
	      gfx_param.short_cone_base, 0);
    glColor3d(0, 0, 1);
    draw_pipe(p0, Eigen::Vector3d(pz.translation()[0],
				  pz.translation()[1],
				  pz.translation()[2]),
	      gfx_param.short_cone_base, 0);
  }
  
  taoDNode /*const*/ * parent(node->getDParent());
  if (parent) {
    // draw cylinder from parent's global frame to node's home frame
    deFrame home;
    home.multiply(*parent->frameGlobal(), *node->frameHome());
    
    if ((prev_tick != tick) && (verbosity >= 2)) {
      cout << "draw_tree(" << (void*) node << ")\n"
	   << "  parent global:    " << *parent->frameGlobal() << "\n"
	   << "  node home local:  " << *node->frameHome() << "\n"
	   << "  node home global: " << home << "\n";
    }
    glColor3d(0.8, 0.8, 0.6);
    draw_pipe(Eigen::Vector3d(parent->frameGlobal()->translation()[0],
			      parent->frameGlobal()->translation()[1],
			      parent->frameGlobal()->translation()[2]),
	      p0,
	      gfx_param.link_radius, gfx_param.link_radius);
  }
  
  if (node->center()) {
    deFrame com;
    com.translation() = *node->center();
    com.multiply(*node->frameGlobal(), deFrame(com));
    
    if ((prev_tick != tick) && (verbosity >= 2)) {
      cout << "  com local:        " << *node->center() << "\n"
	   << "  node global:      " << *node->frameGlobal() << "\n"
	   << "  com global:       " << com.translation() << "\n";
    }
    
    Eigen::Vector3d const p1(com.translation()[0],
			     com.translation()[1],
			     com.translation()[2]);
    vp_tao.UpdateBounds(p1[0], p1[1], p1[2]);
    
    // cone from node's global frame to COM
    glColor3d(0.8, 0.6, 0.8);
    draw_pipe(Eigen::Vector3d(node->frameGlobal()->translation()[0],
			      node->frameGlobal()->translation()[1],
			      node->frameGlobal()->translation()[2]),
	      Eigen::Vector3d(com.translation()[0],
			      com.translation()[1],
			      com.translation()[2]),
	      gfx_param.short_cone_base, 0);
    
    // sphere on COM
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(p1[0], p1[1], p1[2]);
    glutSolidSphere(gfx_param.com_radius, 20, 16);
    glPopMatrix();
  }
  
  taoJointRevolute const * joint(dynamic_cast<taoJointRevolute const *>(node->getJointList()));
  if (joint) {
    deFrame lp0, lp1;
    lp0.translation()[0] = 0;
    lp0.translation()[1] = 0;
    lp0.translation()[2] = 0;
    lp1.translation()[0] = 0;
    lp1.translation()[1] = 0;
    lp1.translation()[2] = 0;
    bool ok(true);
    if (TAO_AXIS_X == joint->getAxis()) {
      lp0.translation()[0] = -gfx_param.joint_length;
      lp1.translation()[0] =  gfx_param.joint_length;
    }
    else if (TAO_AXIS_Y == joint->getAxis()) {
      lp0.translation()[1] = -gfx_param.joint_length;
      lp1.translation()[1] =  gfx_param.joint_length;
    }
    else if (TAO_AXIS_Z == joint->getAxis()) {
      lp0.translation()[2] = -gfx_param.joint_length;
      lp1.translation()[2] =  gfx_param.joint_length;
    }
    else {
      ok = false;
    }
    if (ok) {
      deFrame gp0, gp1;
      gp0.multiply(*node->frameGlobal(), lp0);
      gp1.multiply(*node->frameGlobal(), lp1);
      if (( ! jspace_state) || (jspace_state->force_.size() <= node->getID())) {
	glColor3d(1, 1, 1);
      }
      else {
	double const com_delta(jspace_state->force_[node->getID()]);
	if (fabs(com_delta) < 0.1) {
	  glColor3d(0.5 + fabs(com_delta) / 0.2, 1, 1 - 10 * fabs(com_delta));
	}
	else if (fabs(com_delta) < 1.0) {
	  glColor3d(1, 0, 0.5 + 0.5 * com_delta);
	}
	else if (com_delta < 0) {
	  glColor3d(1, 0, 0);
	}
	else {
	  glColor3d(1, 0, 1);
	}
      }
      draw_pipe(Eigen::Vector3d(gp0.translation()[0],
				gp0.translation()[1],
				gp0.translation()[2]),
		Eigen::Vector3d(gp1.translation()[0],
				gp1.translation()[1],
				gp1.translation()[2]),
		gfx_param.joint_radius, gfx_param.joint_radius);
    }
  }
  
  for (taoDNode * child(node->getDChild()); child != 0; child = child->getDSibling())
    draw_tree(child);
  
  prev_tick = tick;
}


static void draw_transform(jspace::Transform const & gframe)
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMultMatrixd(gframe.matrix().data());
  glColor3d(1, 0, 0);
  draw_pipe(Eigen::Vector3d::Zero(), Eigen::Vector3d(gfx_param.short_cone_length, 0, 0), gfx_param.short_cone_base, 0);
  glColor3d(0, 1, 0);
  draw_pipe(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, gfx_param.short_cone_length, 0), gfx_param.short_cone_base, 0);
  glColor3d(0, 0, 1);
  draw_pipe(Eigen::Vector3d::Zero(), Eigen::Vector3d(0, 0, gfx_param.short_cone_length), gfx_param.short_cone_base, 0);
  glPopMatrix();
}


static void draw_jspace(jspace::Model const & model)
{
  for (size_t ii(0); ii < model.getNDOF(); ++ii) {
    taoDNode * node(model.getNode(ii));
    if ( ! node) {
      continue;
    }
    jspace::Transform gframe;
    if (model.getGlobalFrame(node, gframe)) {
      draw_transform(gframe);
    }
    if (model.computeGlobalCOMFrame(node, gframe)) {
      glColor3d(0.8, 0.6, 0.8);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslated(gframe.translation().x(), gframe.translation().y(), gframe.translation().z());
      glutSolidSphere(gfx_param.com_radius, 20, 16);
      glPopMatrix();
    }
  }
}


static void draw_global_axes()
{
  glColor3d(1, 0, 0);
  draw_pipe(Eigen::Vector3d::Zero(),
	    Eigen::Vector3d(gfx_param.long_cone_length, 0, 0),
	    0, gfx_param.long_cone_base);
  glColor3d(0, 1, 0);
  draw_pipe(Eigen::Vector3d::Zero(),
	    Eigen::Vector3d(0, gfx_param.long_cone_length, 0),
	    0, gfx_param.long_cone_base);
  glColor3d(0, 0, 1);
  draw_pipe(Eigen::Vector3d::Zero(),
	    Eigen::Vector3d(0, 0, gfx_param.long_cone_length),
	    0, gfx_param.long_cone_base);
}


void draw()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);	// Arghl!!! Contrary to doc, this
				// needs to be done at each iteration,
				// not just once during init.
  
  // global specular and shininess values
  static GLfloat global_spec[] = { 1.0, 1.0, 1.0, 1.0 };
  static GLfloat global_shin[] = { 50.0 };
  glMaterialfv(GL_FRONT, GL_SPECULAR,  global_spec);
  glMaterialfv(GL_FRONT, GL_SHININESS, global_shin);
  
  // the rest is done via glColorXx()...
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  
  static GLfloat l_pos[] = { 1.0, 1.0, 1.0, 0.0 };
  
  vp_tao.PushOrtho(0.5);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glLightfv(GL_LIGHT0, GL_POSITION, l_pos);
  
  gltrackball_rotate(trackball);
  glRotatef(-90, 1.0, 0.0, 0.0);
  glRotatef(-90, 0.0, 0.0, 1.0);
  
  draw_global_axes();
  
  draw_tree(model->_getKGMTree()->root);
  
  vp_tao.Pop();
  
  // // jspace::Matrix Jacobian;
  // // taoDNode * end_effector(model->getNode(6));
  // // if (end_effector && model->computeJacobian(end_effector, Jacobian)) {
  // //   jspace::pretty_print(Jacobian, cout, "Jacobian", "  ");
  // // }

  // vp_jspace.PushOrtho(0.5);
  
  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();
  
  // glLightfv(GL_LIGHT0, GL_POSITION, l_pos);
  
  // gltrackball_rotate(trackball);
  // glRotatef(-90, 1.0, 0.0, 0.0);
  // glRotatef(-90, 0.0, 0.0, 1.0);
  
  // draw_global_axes();
  
  // draw_jspace(*model);
  
  // vp_jspace.Pop();
  
  glFlush();
  glutSwapBuffers();
}


void handle(int signum)
{
  switch (signum) {
  case SIGINT:
  case SIGHUP:
  case SIGTERM:
    exit(EXIT_SUCCESS);
  }
  exit(EXIT_FAILURE);
}


void usage(ostream & os)
{
  os << "   -h               help (this message)\n"
     << "   -v               increase verbosity\n"
     << "   -n <iterations>  run without graphics for the number of iterations, then quit\n"
     << "   -s <SAI file>    load SAI file (takes precedence)\n";
}


void parse_options(int argc, char ** argv)
{
  for (int ii(1); ii < argc; ++ii) {
    if ((strlen(argv[ii]) < 2) || (argv[ii][0] != '-')) {
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
    
    switch (argv[ii][1]) {
    case 'h':
      usage(cout);
      exit(EXIT_SUCCESS);
    case 'v':
      ++verbosity;
      if ((strlen(argv[ii]) > 2) && ('v' == argv[ii][2]))
	++verbosity;
      if ((strlen(argv[ii]) > 3) && ('v' == argv[ii][3]))
	++verbosity;
      break;
    case 'n':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-n requires an argument (see -h for more info)");
      }
      {
	istringstream is(argv[ii]);
	is >> n_iterations;
	if ( ! is) {
	  usage(cerr);
	  errx(EXIT_FAILURE, "problem reading n_iterations from \"%s\"", argv[ii]);
	}
      }
      warnx("n_iterations set to %d\n", n_iterations);
      break;
    case 's':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-s requires an argument (see -h for more info)");
      }
      sai_filename = argv[ii];
      break;
    default:
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
  }
}
