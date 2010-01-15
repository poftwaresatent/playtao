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
#include <wbc/util/urdf_to_tao.hpp>
#include <wbc/util/dump.hpp>
#include <wbcnet/log.hpp>

#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoJoint.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoVar.h>
#include <tao/utility/TaoDeMassProp.h>

#include <gfx/wrap_glut.hpp>
#include <gfx/Viewport.hpp>
#include <gfx/Mousehandler.hpp>
#include <gfx/gltrackball.h>

#include <boost/shared_ptr.hpp>

#include <fstream>
#include <err.h>
#include <signal.h>
#include <cstring>

#ifdef HAVE_URDF
# include <urdf/model.h>
#endif

using namespace gfx;
using namespace boost;
using namespace std;

#define USE_DEPTH_BUFFER


static taoNodeRoot * root(0);
static deVector3 gravity(0, 0, -9.81);

static const unsigned int timer_delay(100);
static size_t tick(0);
static trackball_state * trackball;
static int left_button_down(0);
static int winwidth(400);
static int winheight(400);

static bool step(true);
static bool continuous(false);
static int verbosity(1);
static string sai_filename("");
static string urdf_filename("");
static string filter_filename("");
static int n_iterations(-1);	// -1 means graphics mode, user presses 'q' to quit

static void parse_options(int argc, char ** argv);
static void init_glut(int * argc, char ** argv, int width, int height);
static void update_simulation();
static void reshape(int width, int height);
static void draw();
static void keyboard(unsigned char key, int x, int y);
static void timer(int handle);
static void mouse(int button, int state, int x, int y);
static void motion(int x, int y);
static void cleanup(void);
static void handle(int signum);

static taoNodeRoot * create_pendulum();


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
  
  try {
    
    if ( ! sai_filename.empty()) {
      cout << "loading SAI file " << sai_filename << "\n";
      root = parse_sai_xml_file(sai_filename.c_str());
    }
    
    else if ( ! urdf_filename.empty()) {
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
      root = parse_urdf_file(urdf_filename.c_str(), root_name, link_filter.get(),
			     &id_to_link_name, &id_to_joint_name);
    }
    
    else {
      cout << "creating builtin TAO tree\n";
      root = create_pendulum();
    }
    
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "EXCEPTION: %s", ee.what());
  }
  
  if (verbosity >= 1) {
    wbc::dump_tao_tree(cout, root, "FINAL  ", false, &id_to_link_name, &id_to_joint_name);
  }
  
  if (0 == root)
    errx(EXIT_FAILURE, "oops, no TAO root after all that effort?");
  
  taoDynamics::initialize(root);
  
  if (n_iterations < 0) {
    trackball = gltrackball_init();
    init_glut(& argc, argv, winwidth, winheight);
    glutMainLoop();
  }
  else {
    for (int ii(0); ii < n_iterations; ++ii) {
      update_simulation();
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


void update_simulation()
{
  taoDynamics::fwdDynamics(root, &gravity);
  
  static deFloat const dt(0.001);
  for (size_t ii(0); ii < 10; ++ii) {
    taoDynamics::integrate(root, dt);
    taoDynamics::updateTransformation(root);
  }
  
  ++tick;
  if (verbosity >= 2) {
    wbc::dump_tao_tree(cout, root, "", true, 0, 0);
  }
}


void timer(int handle)
{
  if (step || continuous) {
    if (step)
      step = false;
    update_simulation();
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
  if (0 != trackball) {
    free(trackball);
  }
  delete root;
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
  
  draw_tree(root);
  
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


static taoNode * create_ball(taoDNode * parent, int id,
			     double len, double mass, double radius, double qinit)
{
  deFrame com_frame(0, 0, 0);
  deMassProp mp;
  mp.sphere(mass, radius, &com_frame);
  deFrame home_frame(0, 0, -len);
  taoNode * ball(new taoNode(parent, &home_frame));
  mp.get(ball->mass(), ball->center(), ball->inertia());
  ball->setID(id);
  
  taoVarDOF1 * dof(new taoVarDOF1());
  taoJoint * joint(new taoJointRevolute(TAO_AXIS_X));
  joint->setDVar(dof);
  joint->reset();
  joint->setDamping(0.0);
  joint->setInertia(0.0);
  dof->_Q = qinit;
  dof->_dQ = 0;
  dof->_ddQ = 0;
  dof->_Tau = 0;
  
  ball->addJoint(joint); 
  ball->addABNode();
  
  return ball;
}


taoNodeRoot * create_pendulum()
{
  deFrame global_frame(0, 0, 0);
  
  // Weird! You pass a pointer but it gets used to copy-construct a frame...
  taoNodeRoot * root(new taoNodeRoot(&global_frame));
  taoNode * big_ball(create_ball(root, 1, 1, 1, 0.2, M_PI/4));
  taoNode * small_ball(create_ball(big_ball, 2, 0.25, 0.2, 0.05, -M_PI/4));
  taoNode * mini_ball(create_ball(small_ball, 3, 0.125, 0.04, 0.01, M_PI/4));
  
  return root;
}


static void usage(ostream & os)
{
  os << "   -h               help (this message)\n"
     << "   -v               increase verbosity\n"
     << "   -n <iterations>  run without graphics for the number of iterations, then quit\n"
     << "   -s <SAI file>    load SAI file (takes precedence)\n"
     << "   -u <URDF file>   load URDF file (unless a SAI file was specified, too)\n"
     << "   -f <filter file> load a link filter for the URDF conversion (only used with -u)\n";
}


static void parse_options(int argc, char ** argv)
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
	errx(EXIT_FAILURE, "-n requires an option (see -h for more info)");
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
	errx(EXIT_FAILURE, "-s requires an option (see -h for more info)");
      }
      sai_filename = argv[ii];
      break;
    case 'u':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-u requires an option (see -h for more info)");
      }
      urdf_filename = argv[ii];
      break;
    case 'f':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-f requires an option (see -h for more info)");
      }
      filter_filename = argv[ii];
      break;
    default:
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
  }
}
