// CS3241Lab1.cpp : Defines the entry point for the console application.
#include <iostream>
#include <stdbool.h>
#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

using namespace std;

//#define M_PI 3.141592654
#define CULL_NONE 0
#define CULL_FRONT 1
#define CULL_BACK 2

myObjType myObj;

// global variables
char initialFilename[255];

bool m_Smooth = true;
bool m_Highlight = false;
bool m_OriginalObj = true;
double m_Thickness = 0.0;

int m_CullFace = CULL_NONE;
GLfloat angle = 0;   /* in degrees */
GLfloat angle2 = 0;   /* in degrees */
GLfloat zoom = 1.0;
int mouseButton = 0;
int moving, startx, starty;

#define NO_OBJECT 4;
int current_object = 0;

using namespace std;

void setupLighting()
{
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);

	// Lights, material properties
    GLfloat	ambientProperties[]  = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat	diffuseProperties[]  = {0.8f, 0.8f, 0.8f, 1.0f};
    GLfloat	specularProperties[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat lightPosition[] = {-100.0f,100.0f,100.0f,1.0f};
	
    glClearDepth( 1.0 );

	glLightfv( GL_LIGHT0, GL_POSITION, lightPosition);
	
    glLightfv( GL_LIGHT0, GL_AMBIENT, ambientProperties);
    glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
    glLightfv( GL_LIGHT0, GL_SPECULAR, specularProperties);
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

	// Default : lighting
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
}

void display(void)
{
	float mat_specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	float mat_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	float mat_ambient_color[] = { 0.8f, 0.8f, 0.2f, 1.0f };
	float mat_diffuse[] = { 0.1f, 0.5f, 0.8f, 1.0f };
	float shininess = 20;
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
		gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
		glRotatef(angle2, 1.0, 0.0, 0.0);
		glRotatef(angle, 0.0, 1.0, 0.0);
		glScalef(zoom, zoom, zoom);
        myObj.draw();
	glPopMatrix();
	glutSwapBuffers ();
}

void keyboard (unsigned char key, int x, int y)
{
	char filename[256];
    tIdx t;

	switch (key) {
	case 'p':
	case 'P':
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		break;			
	case 'w':
	case 'W':
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		break;			
	case 'v':
	case 'V':
		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
		break;			
	case 's':
	case 'S':
        m_Smooth = !m_Smooth;
        if (m_Smooth) {
            glShadeModel(GL_SMOOTH);
            cout << "Shading: SMOOTH" << endl;
        } else {
            glShadeModel(GL_FLAT);
            cout << "Shading: FLAT" << endl;
        }
        myObj.setSmoothShading(m_Smooth);
        break;
    case 'd':
    case 'D':
        m_CullFace = (m_CullFace + 1) % 3;
        switch (m_CullFace) {
        case CULL_NONE:
            glDisable(GL_CULL_FACE);
            cout << "Culling: NONE" << endl;
            break;
        case CULL_FRONT:
            glEnable(GL_CULL_FACE);
            glCullFace(GL_FRONT);
            cout << "Culling: FRONT" << endl;
            break;
        case CULL_BACK:
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            cout << "Culling: BACK" << endl;
            break;
        default:
            break;
        }
        break;
	case 'h':
	case 'H':
		m_Highlight = !m_Highlight;
		break;
    case 'r':
    case 'R':
        myObj.reset();
        myObj.readFile(initialFilename);
        myObj.setSmoothShading(m_Smooth);
        cout << "Reset Model" << endl;
        break;
    case 'i':
    case 'I':
        cout << "Enter the STL filename you want to read:";
        cin >> filename;
        myObj.readSTL(filename);
        break;
	case 'o':
	case 'O':
		cout << "Enter the filename you want to write:";
        cin >> filename;
        myObj.writeFile(filename);
		break;
	case '1':
	case '2':
	case '3':
	case '4':
		current_object = key - '1';
		break;

    case 'n':
    case 'N':
        cout << "Number of components: " << myObj.countComponents() << endl;
        break;

    case 't':
    case 'T':
        myObj.clearFocus();
        do {
            cout << "Enter a triangle to focus on (0 to quit):";
            cin >> t;
            if (t == 0) {
                continue;
            }
            if (myObj.addFocus(t)) {
                cout << "Selected Triangle: " << t << endl;
            } else {
                cout << "Invalid Triangle" << endl;
            }
        } while (t != 0);
        break;
    case '-':
        m_Thickness -= 1.0;
        myObj.setThickness(m_Thickness);
        cout << "Thickness: " << m_Thickness << endl;
        break;
    case '=':
        m_Thickness += 1.0;
        myObj.setThickness(m_Thickness);
        cout << "Thickness: " << m_Thickness << endl;
        break;
    case '_':
        m_Thickness -= 10.0;
        myObj.setThickness(m_Thickness);
        cout << "Thickness: " << m_Thickness << endl;
        break;
    case '+':
        m_Thickness += 10.0;
        myObj.setThickness(m_Thickness);
        cout << "Thickness: " << m_Thickness << endl;
        break;

    case 'x':
    case 'X':
        myObj.computeIntersections();
        cout << "Computed Intersections" << endl;
        break;
    case 'c':
    case 'C':
        myObj.repairIntersections();
        cout << "Fixed Triangulation" << endl;
        break;
    case 'z':
    case 'Z':
        myObj.removeHiddenFaces();
        cout << "Removed Hidden Faces" << endl;
        break;

	case 'Q':
	case 'q':
		exit(0);
	break;

	default:
	break;
	}

	glutPostRedisplay();
}



void
mouse(int button, int state, int x, int y)
{
  if (state == GLUT_DOWN) {
	mouseButton = button;
    moving = 1;
    startx = x;
    starty = y;
  }
  if (state == GLUT_UP) {
	mouseButton = button;
    moving = 0;
  }
}

void motion(int x, int y)
{
  if (moving) {
	if(mouseButton==GLUT_LEFT_BUTTON)
	{
		angle = angle + (x - startx);
		angle2 = angle2 + (y - starty);
	}
	else zoom += ((y-starty)*0.001);
    startx = x;
    starty = y;
	glutPostRedisplay();
  }
}

int main(int argc, char **argv)
{
	cout<<"CS3242 "<< endl<< endl;
	cout << "Enter the filename you want to open:";
    cin >> initialFilename;
    myObj.readFile(initialFilename);
    myObj.setSmoothShading(m_Smooth);

	//cout << "1-4: Draw different objects"<<endl;
	cout << "S: Toggle Smooth Shading"<<endl;
    cout << "S: Toggle Face Culling"<<endl;
	cout << "H: Toggle Highlight"<<endl;
	cout << "W: Draw Wireframe"<<endl;
	cout << "P: Draw Polygon"<<endl;
	cout << "V: Draw Vertices"<<endl;
    cout << "I: Read STL File" <<endl; // << Optional Task!
    cout << "O: Write OBJ File"<<endl;

    cout << "-/_: Decrease Model Thickness By (-1/-10)"<<endl;
    cout << "=/+: Increase Model Thickness By (+1/+10)"<<endl;

    cout << "T: Select Triangles"<<endl;

    cout << "X: Compute Self-Intersections"<<endl;
    cout << "C: Fix Triangulation (Run after \"Compute Self-Intersections\")"<<endl;
    cout << "Z: Remove Hidden Faces (Run after \"Fix Triangulation\")"<<endl;
    cout << "R: Reset Model"<<endl;

	cout << "Q: Quit" <<endl<< endl;

	cout << "Left mouse click and drag: rotate the object"<<endl;
	cout << "Right mouse click and drag: zooming"<<endl;

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize (600, 600);
	glutInitWindowPosition (50, 50);
	glutCreateWindow ("CS3241 Assignment 3");
	glClearColor (1.0,1.0,1.0, 1.0);
	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	setupLighting();
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST); 
	glDepthMask(GL_TRUE);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(
        40.0,   // field of view in degree
        1.0,    // aspect ratio
        1.0,  // Z near
        80.0);  //Z far
	glMatrixMode(GL_MODELVIEW);
	glutMainLoop();

	return 0;
}
