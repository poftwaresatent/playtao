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

#include "parse.hpp"
#include "robot_api.hpp"
#include <wbc/util/urdf_to_tao.hpp>
#include <wbc/util/dump.hpp>
#include <wbc/util/tao_util.hpp>
#include <wbc/core/RobotAPI.hpp>
#include <wbc/core/RobotFactory.hpp>
#include <wbc/core/Plugin.hpp>
#include <wbc/bin/builtin.hpp>
#include <wbcnet/log.hpp>

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoVar.h>
#include <tao/utility/TaoDeMassProp.h>

#include <saimatrix/SAIVector.h>

#include <gfx/wrap_glut.hpp>
#include <gfx/Viewport.hpp>
#include <gfx/Mousehandler.hpp>
#include <gfx/gltrackball.h>

#include <boost/shared_ptr.hpp>

#include <fstream>
#include <err.h>
#include <signal.h>
#include <cstring>
#include <sys/time.h>


using namespace gfx;
using namespace boost;
using namespace std;

#undef USE_DEPTH_BUFFER


static TAOContainer * tao_container(0);
static taoNodeRoot * tao_root(0);	// this gets deleted for us by the TAOContainer dtor
static deVector3 gravity(0, 0, -9.81);
static int ndof(0);
static wbc::Extensions * wbc_extensions(0);
static std::string robot_spec("");
static std::string servo_spec("");
static wbc::RobotAPI * robot_api(0);
static wbc::BidirectionalRobotAPI * servo_api(0);
static std::string transform_filename("");
static std::ofstream * transform_file(0);
static double servo_rate(500.0); // Hz
static int simul_rate(10);	 // multiple of servo_rate
static double simul_dt(-1);	 // 1.0s / servo_rate / simul_rate
static bool ros_param_mode(false);

static unsigned int timer_delay(100);
static size_t tick(0);
static trackball_state * trackball;
static int left_button_down(0);
static int winwidth(400);
static int winheight(400);

static bool step(true);
static bool continuous(false);
static int verbosity(0);
static string sai_filename("");
static string urdf_filename("");
static string filter_filename("");
static int n_iterations(-1);	// -1 means graphics mode, user presses 'q' to quit

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
  
  wbcnet::configure_logging();
  std::vector<std::string> id_to_link_name; // only for URDF -> TAO conversion though...
  std::vector<std::string> id_to_joint_name; // only for URDF -> TAO conversion though...
  
  parse_options(argc, argv);
  wbcnet::manual_logging_verbosity(verbosity);

  try {
    
    wbc_extensions = wbc::load_extensions(0);
    wbc_extensions->AddRobot("foo", create_robot_factory());
    
    if ( ! robot_spec.empty()) {
      robot_api = wbc_extensions->robot_registry->parseCreate(robot_spec, 0);
    }
    if ( ! servo_spec.empty()) {
      servo_api = wbc_extensions->robot_registry->parseCreateBidirectional(servo_spec, 0);
    }
    if (robot_api && servo_api) {
      errx(EXIT_FAILURE, "you specified -R and -S, that makes no sense");
    }
    
    if ( ! sai_filename.empty()) {
      if ((( ! urdf_filename.empty()) || ros_param_mode)) {
	throw runtime_error("please use only one of the -s, -u, or -p options");
      }
      cout << "loading SAI file " << sai_filename << "\n";
      tao_container = parse_sai_xml_file(sai_filename.c_str());
      tao_root = tao_container->getRoot();
    }
    
    else if ( ! urdf_filename.empty()) {
      if (ros_param_mode) {
	throw runtime_error("please use only one of the -s, -u, or -p options");
      }
      shared_ptr<urdf_to_tao::FlatFileLinkFilter> link_filter;
      string root_name("world");
      if ( ! filter_filename.empty()) {
	cout << "loading link filter file " << filter_filename << "\n";
	link_filter.reset(new urdf_to_tao::FlatFileLinkFilter());
	link_filter->Load(filter_filename);
	root_name = link_filter->GetRootName();
      }
      cout << "root name is " << root_name << "\n"
	   << "loading URDF file " << urdf_filename << "\n";      
      tao_container = parse_urdf_file(urdf_filename.c_str(), root_name, link_filter.get(),
				      &id_to_link_name, &id_to_joint_name);
      tao_root = tao_container->getRoot();
    }
    
    else if (ros_param_mode) {
      cout << "ROS parameter mode\n";
      tao_container = parse_ros_parameter("/pr2_stanford_wbc/", "/robot_description");
      tao_root = tao_container->getRoot();
    }
    
    else {
      usage(cerr);
      errx(EXIT_FAILURE, "you have to specify one of the -s, -u, or -p options");
    }
    
  }
  catch (std::exception const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  if (0 == tao_root)
    errx(EXIT_FAILURE, "oops, no TAO root after all that effort?");
  
  if (verbosity >= 1) {
    wbc::dump_tao_tree(cout, tao_root, "FINAL  ", false, &id_to_link_name, &id_to_joint_name);
  }
  
  ndof = wbc::countDegreesOfFreedom(tao_root);
  taoDynamics::initialize(tao_root);
  
  if (n_iterations < 0) {
    trackball = gltrackball_init();
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
  
  return 0;
}


void init_glut(int * argc, char ** argv,
	       int width, int height)
{
  glutInit(argc, argv);
#ifdef USE_DEPTH_BUFFER
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
  glEnable(GL_DEPTH_TEST);
#else
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
#endif
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);
  
  int handle(glutCreateWindow("playtao"));
  if (0 == handle)
    errx(EXIT_FAILURE, "glutCreateWindow() failed");
  
  glutDisplayFunc(draw);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutTimerFunc(timer_delay, timer, handle);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
}


void reshape(int width, int height)
{
  Subwindow::DispatchResize(screen_point_t(width, height));
  winwidth = width;
  winheight = height;
}


void keyboard(unsigned char key, int x, int y)
{
  switch(key){
  case ' ':
    step = true;
    continuous = false;
    break;
  case 'c':
    step = false;
    continuous = true;
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


static void dump_global_frames(std::ostream & os, taoDNode * node, std::string const & prefix)
{
  os << prefix << "ID " << node->getID() << ": " << *node->frameGlobal() << "\n";
  for (taoDNode * child(node->getDChild()); child != 0; child = child->getDSibling()) {
    dump_global_frames(os, child, prefix);
  }
}


bool update()
{
  SAIVector pos(ndof);
  SAIVector vel(ndof);
  SAIVector tau(ndof);
  struct timeval tstamp;
  
  if (robot_api) {
    if ( ! robot_api->readSensors(pos, vel, tstamp, 0)) {
      cerr << "update(): robot_api->readSensors() failed\n";
      return false;
    }
    if (pos.size() != ndof) {
      cerr << "WARNING in update(): pos.size() is " << pos.size() << " but should be " << ndof << "\n";
      pos.setSize(ndof, true);
    }
    double const * foo(&pos[0]);
    set_node_state(tao_root, foo);
    taoDynamics::updateTransformation(tao_root);
  }

  else {
    if (servo_api) {
      get_node_state(tao_root, &pos[0], &vel[0]);
      if ( ! servo_api->writeSensors(pos, vel, 0)) {
	errx(EXIT_FAILURE, "update(): servo_api->writeSensors() failed");
      }
      if ( ! servo_api->readCommand(tau)) {
	errx(EXIT_FAILURE, "update(): servo_api->readTorques() failed");
      }
    }
    
    for (size_t ii(0); ii < simul_rate; ++ii) {
      taoDynamics::fwdDynamics(tao_root, &gravity);
      if (servo_api) {
	add_node_command(tao_root, &tau[0]);
      }
      taoDynamics::integrate(tao_root, simul_dt);
      taoDynamics::updateTransformation(tao_root);
    }
  }
  
  ++tick;
  if (verbosity >= 2) {
    wbc::dump_tao_tree(cout, tao_root, "", true, 0, 0);
  }
  
  if (transform_file) {
    (*transform_file) << "==================================================\n"
		      << "joint_positions:\t" << pos << "\n"
		      << "link_origins:\n";
    dump_global_frames(*transform_file, tao_root->getDChild(), "  ");
  }
  
  return true;
}


void timer(int handle)
{
  if (step || continuous) {
    if (step)
      step = false;
    if ( ! update()) {
      errx(EXIT_FAILURE, "timer(): update() failed");
    }
  }
  
  Subwindow::DispatchUpdate();
  
  glutSetWindow(handle);
  glutPostRedisplay();
  
  glutTimerFunc(timer_delay, timer, handle);
}


void mouse(int button, int state, int x, int y)
{
  Subwindow::DispatchClick(button, state, screen_point_t(x, y));
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
  Subwindow::DispatchDrag(screen_point_t(x, y));
  if (0 != left_button_down)
    gltrackball_track (trackball, x, y, winwidth, winheight);
}


void cleanup(void)
{
  if (transform_file) {
    // flushing and closing is probably redundant...
    (*transform_file) << flush;
    transform_file->close();
    delete transform_file;
  }
  if (0 != trackball) {
    free(trackball);
  }
  delete tao_container;
  delete robot_api;
  delete wbc_extensions;
}


static void draw_tree(taoDNode /*const*/ * node)
{
  static size_t prev_tick(812379);
  
  taoDNode /*const*/ * parent(node->getDParent());
  if (parent) {
    // draw thick line from parent's global frame to node's home frame
    deFrame home;
    home.multiply(*parent->frameGlobal(), *node->frameHome());
    
    if ((prev_tick != tick) && (verbosity >= 2)) {
      cout << "draw_tree(" << (void*) node << ")\n"
	   << "  parent global:    " << *parent->frameGlobal() << "\n"
	   << "  node home local:  " << *node->frameHome() << "\n"
	   << "  node home global: " << home << "\n";
    }
    
    glLineWidth(3);
    glColor3d(0.5, 0.5, 0.5);
    glBegin(GL_LINES);
    glVertex3d(parent->frameGlobal()->translation()[0],
	       parent->frameGlobal()->translation()[1],
	       parent->frameGlobal()->translation()[2]);
    glVertex3d(home.translation()[0],
	       home.translation()[1],
	       home.translation()[2]);
    glEnd();
  }
  
  if (node->center()) {
    // draw thin line from node's global frame to COM, and thick dot on COM
    deFrame com;
    com.translation() = *node->center();
    com.multiply(*node->frameGlobal(), deFrame(com));
    
    if ((prev_tick != tick) && (verbosity >= 2)) {
      cout << "  com local:        " << *node->center() << "\n"
	   << "  node global:      " << *node->frameGlobal() << "\n"
	   << "  com global:       " << com.translation() << "\n";
    }
    
    glLineWidth(1);
    glColor3d(0.5, 1, 0.5);
    glBegin(GL_LINES);
    glVertex3d(node->frameGlobal()->translation()[0],
	       node->frameGlobal()->translation()[1],
	       node->frameGlobal()->translation()[2]);
    glVertex3d(com.translation()[0],
	       com.translation()[1],
	       com.translation()[2]);
    glEnd();
    
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3d(0, 1, 0.5);
    glVertex3d(com.translation()[0],
	       com.translation()[1],
	       com.translation()[2]);
    glEnd();
  }
  
  for (taoDNode * child(node->getDChild()); child != 0; child = child->getDSibling())
    draw_tree(child);
  
  prev_tick = tick;
}


void draw()
{
  glClearColor(0, 0, 0, 0);
#ifdef USE_DEPTH_BUFFER
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#else
  glClear(GL_COLOR_BUFFER_BIT);
#endif
  
  static Viewport * view(0);
  if ( ! view) {
    view = new Viewport("view", logical_bbox_t(-2, -2, 2, 2), logical_bbox_t(0, 0, 1, 1));
    ////view->SetMousehandler(Viewport::LEFT, m_mouse_meta);
    view->Enable();
  }
  
  view->PushProjection();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gltrackball_rotate(trackball);
  glRotatef(-90, 1.0, 0.0, 0.0);
  glRotatef(-90, 0.0, 0.0, 1.0);
  
  glLineWidth(1);
  glBegin(GL_LINES);
  glColor3d(1, 0, 0);
  glVertex3d(0, 0, 0);
  glVertex3d(1, 0, 0);
  glColor3d(0, 1, 0);
  glVertex3d(0, 0, 0);
  glVertex3d(0, 1, 0);
  glColor3d(0, 0, 1);
  glVertex3d(0, 0, 0);
  glVertex3d(0, 0, 1);
  glEnd();
  
  draw_tree(tao_root);
  
  view->PopProjection();
  
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
     << "   -p               use ROS parameters to initialize robot model\n"
     << "   -n <iterations>  run without graphics for the number of iterations, then quit\n"
     << "   -s <SAI file>    load SAI file (takes precedence)\n"
     << "   -u <URDF file>   load URDF file (unless a SAI file was specified, too)\n"
     << "   -f <filter file> load a link filter for the URDF conversion (only used with -u)\n"
     << "   -R <robot spec>  retrieve joint angles from a robot at each tick\n"
     << "   -S <servo spec>  retrieve torque commands from a servo at each tick\n"
     << "   -t <filename>    write joint angles and link transforms to file at each tick\n";
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
    case 'p':
      ros_param_mode = true;
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
    case 'u':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-u requires an argument (see -h for more info)");
      }
      urdf_filename = argv[ii];
      break;
    case 'f':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-f requires an argument (see -h for more info)");
      }
      filter_filename = argv[ii];
      break;
    case 'R':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-R requires an argument (see -h for more info)");
      }
      robot_spec = argv[ii];
      break;
    case 'S':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-S requires an argument (see -h for more info)");
      }
      servo_spec = argv[ii];
      break;
    case 't':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-t requires an argument (see -h for more info)");
      }
      transform_filename = argv[ii];
      transform_file = new ofstream(argv[ii]);
      if ( ! (*transform_file)) {
	errx(EXIT_FAILURE, "problem opening transform file \"%s\"", argv[ii]);
      }
      break;
    default:
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
  }
  
  simul_dt = 1.0 / servo_rate / simul_rate;
  
  if ( ! servo_spec.empty()) {
    timer_delay = 10;
  }
}
