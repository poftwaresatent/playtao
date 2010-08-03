// hand-copied from Woo et.al. "OpenGL Programming Guide" 3rd edition Addison Wesley 1999
// gcc -Wall -o light light.c -lglut -LGLU

#include <GL/glut.h>
#include <stdlib.h>

static GLUquadricObj * qobj;


void init()
{
  GLfloat mat_spec[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mat_shin[] = { 50.0 };
  GLfloat l_pos[] = { 1.0, 1.0, 1.0, 0.0 };
  GLfloat l_white[] = { 1.0, 1.0, 1.0, 1.0 };
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glShadeModel(GL_SMOOTH);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_spec);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shin);
  glLightfv(GL_LIGHT0, GL_POSITION, l_pos);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, l_white);
  glLightfv(GL_LIGHT0, GL_SPECULAR, l_white);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glRotatef(45.0, 0.7, 0.7, 0.0);
  gluCylinder(qobj, 0.2, 0.2, 1.5, 8, 1);
  glutSolidSphere(1.0, 20, 16);
  glFlush();
}

void reshape(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  if (w <= h) {
    glOrtho(-1.5, 1.5, (-1.5 * h) / w, (1.5 * h) / w, -10.0, 10.0);
  }
  else {
    glOrtho((-1.5 * w) / h, (1.5 * w) / h, -1.5, 1.5, -10.0, 10.0);
  }
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

int main(int argc, char ** argv)
{
  qobj = gluNewQuadric();
  gluQuadricNormals(qobj, GLU_SMOOTH);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutInitWindowPosition(100, 100);
  glutCreateWindow(argv[0]);
  init();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutMainLoop();
  return 0;
}
