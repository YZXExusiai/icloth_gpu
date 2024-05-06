#include<iostream>
#include <fstream>
#include <cmath>
#include "GL/glh_glut.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "simulation.h"
#include <stdio.h>


bool b[256];
int win_w = 600, win_h = 600;
static int level = 1;
using namespace glh;
glut_simple_mouse_interactor object;

float lightpos[4] = { 13, 10.2, 3.2, 0 };
bool wireframe = false;
ImportedModel myModel("cloth2.obj");
ImportedModel myCube("Tcube2.obj");
Mesh* cloth = initial_cloth(myModel);
Mesh* cube = initial_block(myCube);
extern Simulation initial_sim();
extern void drawModel(const Mesh* cloth);
extern void DynamicModel(Simulation& sim);
extern bool push_sim(Simulation& sim, Mesh* cloth);
Simulation sim = initial_sim();
bool isCreatecloth = push_sim(sim, cloth);
bool isCreateblock = push_sim_static(sim, cube);
void initSetting()
{
	b['9'] = false;
	b['d'] = false;
	b['h'] = true;
}
void initOpengl()
{
	glClearColor(1.0, 1.0, 1.0, 1.0);

	// initialize OpenGL lighting
	GLfloat lightPos[] = { 10.0, 10.0, 10.0, 0.0 };
	GLfloat lightAmb[4] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lightDiff[4] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lightSpec[4] = { 1.0, 1.0, 1.0, 1.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, &lightpos[0]);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiff);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);

	//glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL_EXT, GL_SEPARATE_SPECULAR_COLOR_EXT);
	GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}
void begin_window_coords()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, win_w, 0.0, win_h, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}
void end_window_coords()
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

void drawGround()
{
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	glBegin(GL_QUADS);
	glColor3f(1.f, 0.f, 0.f);
	glVertex3f(20, 0, 20);
	glVertex3f(-20, 0, 20);
	glVertex3f(-20, 0, -20);
	glVertex3f(20, 0, -20);
	glEnd();

	glDisable(GL_COLOR_MATERIAL);
}

void draw()
{
	glPushMatrix();
	glScaled(0.4, 0.4, 0.4);
	glRotatef(-90, 1, 0, 0);
	//绘制模型
	drawModel(cloth);
	drawModel(cube);

	glPopMatrix();

	if (b['g'])
		drawGround();
}
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glShadeModel(GL_SMOOTH);

	if (!b['b']) {
		// gradient background
		begin_window_coords();
		glBegin(GL_QUADS);
		glColor3f(0.2, 0.4, 0.8);
		glVertex2f(0.0, 0.0);
		glVertex2f(win_w, 0.0);
		glColor3f(0.05, 0.1, 0.2);
		glVertex2f(win_w, win_h);
		glVertex2f(0, win_h);
		glEnd();
		end_window_coords();
	}

	glMatrixMode(GL_MODELVIEW);

	glLoadIdentity();
	object.apply_transform();

	// draw scene
	if (b['w'])
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glEnable(GL_LIGHTING);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	draw();

	glutSwapBuffers();

}
void idle()
{
	if (b[' '])
		object.trackball.increment_rotation();

	if (b['d'])
	{
		DynamicModel(sim);
	}

	glutPostRedisplay();
}
void resize(int w, int h)
{
	if (h == 0) h = 1;

	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	object.reshape(w, h);

	win_w = w; win_h = h;
}
void quit()
{
	//quitModel();
	exit(0);
}
void mouse(int button, int state, int x, int y)
{
	object.mouse(button, state, x, y);
}
void motion(int x, int y)
{
	object.motion(x, y);
}
void key(unsigned char k, int x, int y) {
	b[k] = !b[k];
	switch (k) {
	case 'q':
		quit();
		break;
	}
}
void main_menu(int i)
{
	key((unsigned char)i, 0, 0);
}
void initMenu()
{
	glutCreateMenu(main_menu);
	glutAddMenuEntry("Toggle animation [d]", 'd');
	glutAddMenuEntry("Toggle wireframe [w]", 'w');
	glutAddMenuEntry("========================", '=');
	glutAddMenuEntry("Quit/q [esc]", '\033');
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

int main(int argc, char** argv)
{
	//ImportedModel myModel("shuttle.obj");
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA | GLUT_STENCIL);
	glutInitWindowSize(win_w, win_h);
	glutCreateWindow("Cloth Simulator");

	initOpengl();
	//初始化模型
	//initModel(argc, argv, dataPath, stFrame, visPath);

	object.configure_buttons(1);
	object.dolly.dolly[2] = -3;
	object.trackball.incr = rotationf(vec3f(1, 1, 0), 0.05);

	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutIdleFunc(idle);
	glutKeyboardFunc(key);
	glutReshapeFunc(resize);

	initMenu();

	initSetting();

	//启动多线程
	//startMultiThreads(animPath);
	glutMainLoop();

	quit();

	return 0;
}

