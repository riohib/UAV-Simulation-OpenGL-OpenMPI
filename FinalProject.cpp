/*
 Author: Riyasat Ohib
 Class: ECE6122
 Last Date Modified: Dec 3, 2019.
 Description: Final Project Source CPP File
 All the Major Functions are in the Header File. This cpp file has the
 main driver functions and MPI communications set up.
 */

#include "finalProject.h"
#include "ECE_Bitmap.h"

// Send location and velocity vector in each direction
const int numElements = 6; // x, y, z, vx, vy, vz
const int numDrones = 16;
const int rcvSize = 6 * 16; // (Main task + 15 UAVs) * numElements

double* rcvbuffer = new double[rcvSize];

double sendBuffer[numElements];

int rank;

//========================== TEXTURE ===================================
// This Code Block Helps load the Texture
#include "ECE_Bitmap.h"

GLuint texture[1];

float x_angle = 0;
float y_angle = 0;
float z_angle = 0;

struct Image {
    unsigned long sizeX;
    unsigned long sizeY;
    char *data;
};

typedef struct Image Image;
BMP inBitmap;
#define checkImageWidth 64
#define checkImageHeight 64

GLubyte checkImage[checkImageWidth][checkImageHeight][3];

void myInit(void)
{
  
    glClearColor(0.5, 0.5, 0.5, 0.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    inBitmap.read("football.bmp");

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // Create Textures
    glGenTextures(2, texture);

    // Setup first texture
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
        GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    glEnable(GL_TEXTURE_2D);

}

//======================================================================
// declareBuzzyType(argc, argv);

//----------------------------------------------------------------------
// Reshape callback
//
// Window size has been set/changed to w by h pixels. Set the camera
// perspective to 45 degree vertical field of view, a window aspect
// ratio of w/h, a near clipping plane at depth 1, and a far clipping
// plane at depth 100. The viewport is the entire window.
//
//----------------------------------------------------------------------
void changeSize(int w, int h)
{
    float ratio = ((float)w) / ((float)h); // window aspect ratio
    glMatrixMode(GL_PROJECTION); // projection matrix is active
    glLoadIdentity(); // reset the projection
    gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
    glMatrixMode(GL_MODELVIEW); // return to modelview mode
    glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}
//----------------------------------------------------------------------
// mainOpenGL  - standard GLUT initializations and callbacks
//----------------------------------------------------------------------
void timerFunction(int id)
{
    glutPostRedisplay();
    MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
    glutTimerFunc(100, timerFunction, 0);
}

//----------------------------------------------------------------------
// Draw the entire scene
//
// We first update the camera location based on its distance from the
// origin and its direction.
//----------------------------------------------------------------------
int i = 0;
double xCoord, yCoord, zCoord;
void renderScene()
{
    // Clear color and depth buffers
    glClearColor(0.5, 5.0, 0.9, 1.0); // background color to green??
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();

    gluLookAt(eye_x, eye_y, eye_z,    //
    center_x, center_y, center_z,
    0.0, 0.0, 1.0);
    clearCameraVariables();
    
    //--- Texture Setup ---
    glBindTexture(GL_TEXTURE_2D, texture[0]);

    glBegin(GL_QUADS);
        glTexCoord2f(1, 1);
        glVertex3f(50.0f, 25.0f, 0.0f);
        glTexCoord2f(0, 1);
        glVertex3f(-50.0f, 25.0f, 0.0f);
        glTexCoord2f(0, 0);
        glVertex3f(-50.0f, -25.0f, 0.0f);
        glTexCoord2f(1, 0);
        glVertex3f(50.0f, -25.0f, 0.0f);
    glEnd();
    glPopMatrix();
    
    //--------- Create the Football Field Zone ---------
    displayFootballField();

    glBindTexture(GL_TEXTURE_2D, 0); // Clear Texture

    glMatrixMode(GL_MODELVIEW);

    // drawSphere Central Sphere
    // drawSphere(); 

    for (int i=1; i < 16; ++i)
    {
        xCoord = rcvbuffer[6 * i]; yCoord = rcvbuffer[6*i + 1]; zCoord = rcvbuffer[6*i +2];
        // std::cout << "x: " << xCoord << " | y: " << yCoord << " | z: " << yCoord ;
        drawUAVs(xCoord, yCoord, zCoord);
    }
    glutSwapBuffers(); // Make it all visible
}

//----------------------------------------------------------------------
// mainOpenGL  - standard GLUT initializations and callbacks
//----------------------------------------------------------------------
void mainOpenGL(int argc, char**argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    glutCreateWindow(argv[0]);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    myInit();
    // Setup lights as needed
    // ...

    // -------- From Chess--------
    // register callbacks
    glutReshapeFunc(changeSize); // window reshape callback
    glutDisplayFunc(renderScene); // (re)display callback
    glutReshapeFunc(reshape);
    glutIdleFunc(update); // incremental update
    glutIgnoreKeyRepeat(1); // ignore key repeat when holding key down
    glutMouseFunc(mouseButton); // process mouse button push/release
    glutMotionFunc(mouseMove); // process mouse dragging motion
    glutKeyboardFunc(processNormalKeys); // process standard key clicks
    glutTimerFunc(100, timerFunction, 0);
    glutSpecialFunc(pressSpecialKey); // process special key pressed
    // Warning: Nonstandard function! Delete if desired.
    glutSpecialUpFunc(releaseSpecialKey); // process special key release

    glutMainLoop();

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
// Main entry point determines rank of the process and follows the 
// correct program path
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
int main(int argc, char**argv)
{
//**************************** HINT BLOCK ******************************
    int numTasks, rank;

    int rc = MPI_Init(&argc, &argv);

    if (rc != MPI_SUCCESS)
    {
        printf("Error starting MPI program. Terminating.\n");
        MPI_Abort(MPI_COMM_WORLD, rc);
    }

    MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    int gsize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &gsize);

    for (int ii = 0; ii < 700; ii++) // Change 2 to 600
        {
        if (rank == 0)
        {
            int disp = rank + 2;
            MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
            mainOpenGL(argc, argv);
        }
        else
        {
            int disp = rank + 11;
            
            if (ii == 0){
                intializeUAVLocation(rank, sendBuffer);
                MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);    
            }
            else
            {
                calcualteUAVsLocation(ii, rank, sendBuffer, rcvbuffer);
                
                populateSendBuffer(rank, sendBuffer, rcvbuffer);   
                
                if (ii == 1){
                    std::this_thread::sleep_for(std::chrono::seconds(5)); // Sleep for 5 seconds
                } 
                MPI_Allgather(sendBuffer, numElements, MPI_DOUBLE, rcvbuffer, numElements, MPI_DOUBLE, MPI_COMM_WORLD);
            }
        }
    }
    return 0;
}
