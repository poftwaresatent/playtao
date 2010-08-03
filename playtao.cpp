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

#include "strutil.hpp"

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

// most of these are currently unused...
static double simul_rate(500.0);
static double simul_dt(0.002);
static double servo_rate(250.0);
static double servo_dt(0.004);
static double gfx_rate(50.0);
static unsigned int gfx_timer_ms(20);
static size_t n_simul_per_servo(2);
static size_t n_simul_per_gfx(10);

static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<jspace::RobotAPI> robot_api;
static GLUquadricObj * qobj(0);

static size_t tick(0);
static trackball_state * trackball;
static int left_button_down(0);
static int winwidth(400);
static int winheight(400);
static gfx::Viewport viewport;

static bool step(true);
static bool continuous(false);
static int verbosity(0);
static string sai_filename("");
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

  try {
    model.reset(load_model());
    
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
  
  return new jspace::Model(kgm_tree, cc_tree);
}


void init_glut(int * argc, char ** argv,
	       int width, int height)
{
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);
  glEnable(GL_DEPTH_TEST);
  
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);
  
  int handle(glutCreateWindow("playtao"));
  if (0 == handle)
    errx(EXIT_FAILURE, "glutCreateWindow() failed");
  
  {
    GLfloat mat_spec[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shin[] = { 50.0 };
    GLfloat l_white[] = { 1.0, 1.0, 1.0, 1.0 };
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shin);
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
  viewport.UpdateShape(width, height);
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
  case 'r':
    try {
      jspace::Model * nm(load_model());
      model.reset(nm);
      viewport.ResetBounds();
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
  jspace::State state;
  size_t const ndof(model->getNDOF());
  
  if ( ! robot_api) {
    state.init(ndof, ndof, 0);
  }
  else {
    jspace::Status status(robot_api->readState(state));
    if ( ! status) {
      cerr << "update(): robot_api->readState() failed: " << status.errstr << "\n";
      return false;
    }
    if (state.position_.size() != ndof) {
      cerr << "update(): WARNING state has " << state.position_.size()
	   << " DOF but should have " << ndof << "\n";
    }
  }
  
  model->update(state);
  
  if (verbosity >= 2) {
    jspace::dump_tao_tree_info(cout, model->_getKGMTree(), "", true);
  }
  
  // if (transform_file) {
  //   (*transform_file) << "==================================================\n"
  // 		      << "joint_positions:\t" << pos << "\n"
  // 		      << "link_origins:\n";
  //   dump_global_frames(*transform_file, tao_tree->root->getDChild(), "  ");
  // }
  
  ++tick;
  return true;
}


void timer(int handle)
{
  if (step || continuous) {
    if (step) {
      step = false;
      if ( ! update()) {
	errx(EXIT_FAILURE, "timer(): update() failed");
      }
    }
    else {
      do {
	if ( ! update()) {
	  errx(EXIT_FAILURE, "timer(): update() failed");
	}
      }  // tick gets incremented in update(), unless it
         // returns false, in which case we abort anyway.
      while (0 != tick % n_simul_per_gfx);
    }
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


static void draw_tree(taoDNode /*const*/ * node)
{
  static size_t prev_tick(812379);
  
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
    
    viewport.UpdateBounds(parent->frameGlobal()->translation()[0],
			  parent->frameGlobal()->translation()[1],
			  parent->frameGlobal()->translation()[2]);
    viewport.UpdateBounds(home.translation()[0],
			  home.translation()[1],
			  home.translation()[2]);
    
    Eigen::Vector3d p0(parent->frameGlobal()->translation()[0],
		       parent->frameGlobal()->translation()[1],
		       parent->frameGlobal()->translation()[2]);
    Eigen::Vector3d p1(home.translation()[0],
		       home.translation()[1],
		       home.translation()[2]);
    static Eigen::Vector3d const ez(0, 0, 1);
    Eigen::Vector3d ezd(p1 - p0);
    double const ezdnorm(ezd.norm());
    if (ezdnorm > 1e-6) {	// hmm... and else?
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
      // we are already in glMatrixMode(GL_MODELVIEW)
      glPushMatrix();
      glMultMatrixd(transform.data());
      glColor3d(0.5, 0.5, 0.5);
      // gluQuadricDrawStyle(qobj, GLU_LINE);
      // glLineWidth(1);
      gluCylinder(qobj, 0.2, 0.2, ezdnorm, 8, 1);
      glPopMatrix();
    }
    
    // draw thick line from parent's global frame to node's home frame
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
    
    viewport.UpdateBounds(node->frameGlobal()->translation()[0],
			  node->frameGlobal()->translation()[1],
			  node->frameGlobal()->translation()[2]);
    viewport.UpdateBounds(com.translation()[0],
			  com.translation()[1],
			  com.translation()[2]);
    
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
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  viewport.PushOrtho();
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  {
    GLfloat l_pos[] = { 1.0, 1.0, 1.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, l_pos);
    glColor3d(1, 1, 0);
    glPushMatrix();
    glTranslatef(l_pos[0], l_pos[1], l_pos[2]);
    glutSolidSphere(0.1, 20, 16);
    glPopMatrix();
  }
  
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
  
  draw_tree(model->_getKGMTree()->root);
  
  viewport.Pop();
  
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
     << "   -s <SAI file>    load SAI file (takes precedence)\n"
     << "   -T <simul:servo:gfx> specify update rates (in Hz) for the servo, the simulation, and the graphics\n";
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
    case 'T':
      ++ii;
      if (ii >= argc) {
	usage(cerr);
	errx(EXIT_FAILURE, "-T requires an argument (see -h for more info)");
      }
      {
	vector<string> token;
	sfl::tokenize(argv[ii], ':', token);
	sfl::token_to(token, 0, simul_rate);
	sfl::token_to(token, 1, servo_rate);
	sfl::token_to(token, 2, gfx_rate);
	if (0 >= servo_rate) {
	  usage(cerr);
	  errx(EXIT_FAILURE, "invalid servo rate: %g Hz (must be > 0)", servo_rate);
	}
	if (simul_rate < servo_rate) {
	  warnx("invalid simulation rate %g Hz adjusted to %g Hz", simul_rate, servo_rate);
	  simul_rate = servo_rate;
	}
	if (gfx_rate > simul_rate) {
	  warnx("invalid graphics rate %g Hz adjusted to %g Hz", gfx_rate, simul_rate);
	  gfx_rate = simul_rate;
	}
	servo_dt = 1.0 / servo_rate;
	simul_dt = 1.0 / simul_rate;
	gfx_timer_ms = ceil(1000 / gfx_rate);
	if (5 >= gfx_timer_ms) {
	  warnx("invalid graphics timer %ud ms adjusted to 5 ms", gfx_timer_ms);
	  gfx_timer_ms = 5;
	}
	n_simul_per_servo = ceil(simul_rate / servo_rate);
	if (0 >= n_simul_per_servo) {
	  n_simul_per_servo = 0;
	}
	n_simul_per_gfx = ceil(simul_rate / gfx_rate);
	if (0 >= n_simul_per_gfx) {
	  n_simul_per_gfx = 0;
	}
      }
      break;
    default:
      usage(cerr);
      errx(EXIT_FAILURE, "problem with option '%s'", argv[ii]);
    }
  }
}
