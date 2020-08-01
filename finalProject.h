/*
 Author: Riyasat Ohib
 Class: ECE6122
 Last Date Modified: Dec 3, 2019.
 Description: Final Project Header File
 All the Major Functions for the implementations are here.
 All Kinematics and OpenGL drawing functions are here.
 */
#ifndef FP_H
#define FP_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
// #include <GL/glut.h>
#include <chrono>
#include <thread>
#include <random>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define ESC 27
#define pi 3.141592653589793238462643383279502884L /* pi */

#include "ECE_Bitmap.h"

double mass = 1.0f;
double radius = 10.0;
double sphereCenter = 50.0f;
//=================== UAV variables ======================//
double m = 1; //Mass = 1kg
// double t = 1; // time in kinematics equations
double uavCoords[3];
double uavVelocity[3];


//==================== camera variables =======================//
bool pressD = false;
float eyeZ = 0.0f;
float deltaMove = 0.0f;          // initially camera doesn't move
float deltaMoveH = 0.0f;
float deltaMoveZ = 0.0f;

float lx = 0.0, ly = 1.0; // camera points initially along y-axis
float angle = 0.0;   //angle of rotation for the camera direction
float deltaAngle = 0.0;  // additional angle change when dragging


float R = 0.54, G = 0.54, B = 0.529;

float wr = 0.54, wg = 0.54, wb = 0.529;
float br = 0.588, bg = 0.294, bb = 0.0;

// Mouse drag control
int isDragging = 0; // true when dragging
int xDragStart = 0; // records the x-coordinate when dragging starts

// //=================== FORWARD DECLARATIONS =======================//
// void processNormalKeys(unsigned char key, int xx, int yy);
// void pressSpecialKey(int key, int xx, int yy);
// void releaseSpecialKey(int key, int x, int y);

//================================================================//
// float x = 0.0, y = 0.0, z = 0.0; 
float eye_x = 0.0, eye_y = -85.0, eye_z = 95;
float center_x = 0, center_y = 0, center_z = 45;



//======================== DRAW STUFF Field and UAV ==================================//
void displayFootballField()
{
    /*
    OpenGL Function to create the Football field for display.
    Called Inside Render Scene
    Input: None
    Output: None 
    */
    glColor3f(0.0, 0.7, 0.8);
    glBegin(GL_QUADS);
    glVertex3f(-50.0, -25.0, 0.0);
    glVertex3f(-50.0, 25.0, 0.0);
    glVertex3f(50.0, 25.0, 0.0);
    glVertex3f(50.0, -25.0, 0.0);
    glEnd();
}


void drawUAVs(float ux, float uy, float uz)
{
    /*
    OpenGL Function to draw the UAVs for display.
    Called Inside Render Scene
    Input: Floating point x, y, z coordinates of the UAV locations to be drawn.
    Output: None 
    */
    glColor3f(1,0,1); // set drawing color to white
    glPushMatrix();
    glTranslatef(ux, uy, uz);
    glScalef(0.8, 0.8, 0.8);
    glutSolidIcosahedron();
    glPopMatrix();
}

void drawSphere()
{   
    /*
    OpenGL Function to draw a Sphere at the center for display/Debug.
    Called Inside Render Scene
    Input: Floating point x, y, z coordinates of the UAV locations to be drawn.
    Output: None 
    */

    // Draw body (a 20x20 spherical mesh of radius 0.75 at height 0.75)
    glColor3f(0,1,1); // set drawing color to white
    glPushMatrix();
    glTranslatef(0.0, 0.0, sphereCenter);
    glutWireSphere(10, 20, 20);
    glPopMatrix();
}

// ========================= INITIALIZE UAV LOCATION ========================= //

void populateSendBuffer(int rank, double buffer[6], double rcvbuffer[16*6])
{
    /*
    Function to Extract Message from Received Buffer from AllGather and Populate the sendBuffer based on rank.
    Input: Int Current Rank, doulble Array of Buffer of Size 6, doulble Array of ReceiveBuffer of Size 6*16
    Output: None (Mutates the Buffers through Pass by Reference).
    */
    buffer[0] = rcvbuffer[6 * rank]; buffer[1] = rcvbuffer[6*rank + 1]; buffer[2] = rcvbuffer[6*rank +2];
    buffer[3] = rcvbuffer[6 * rank + 3]; buffer[4] = rcvbuffer[6*rank + 4]; buffer[5] = rcvbuffer[6*rank + 5];   
}

// --------------------- UAV INITIALIZATION ----------------------- //
double ux0 = 0.0f, uy0 =0.0f,  uz0 = 0.0f;

void intializeUAVLocation(int rank, double buffer[6])
{   
    /*
    Function to Initialize the UAV location through OpenGL's translation function.
    Draws the UAV's at the respective set point in the football field.
    Input: Int Rank, Array of Doubles of Size 6 named buffer.
    Output: None (Mutates Buffer).
    */
    int disp = rank -1 ;
    if (rank < 6)
    {
        ux0 = -50 + (disp * 25); uy0 = -25; uz0 = 0.0;
    }
    else if (rank < 11)
    {
        ux0 = -50 + ((disp % 5) * 25); uy0 = 0.0; uz0 = 0.0;
    }
    else
    {
        ux0 = -50 + ((disp % 5) * 25); uy0 = 25.0; uz0 = 0.0;
    }
    buffer[0] = ux0; buffer[1] = uy0; buffer[2] = uz0; 
    // buffer[3] = vx0; buffer[4] = vy0; buffer[5] = vz0;
    buffer[3] = 0.0; buffer[4] = 0.0; buffer[5] = 0.0;  
}

//================================ Virtual Point ==================================//
double u = 0; double itr = 0.0;
double X = 0; double Y = 0; double Z = 0;
double xx, yy, zz;
double degree = 0;
double rotCoord[3];
double uDelta = 0.0;

// ========================= CALCULATE UAV NEXT LOCATION ========================= //
// ----------------------------- VARIABLE DECLATIONS ------------------------------//
double x_0, y_0, z_0;
double vx0 = 0.0f, vy0 =0.0f,  vz0 = 0.0f;

double dist = 0;
double thrust[3];
double prevXDist = 1000; double prevYDist = 1000; double prevZDist = 1000;
int signX = 1 ; int signY = 1; int signZ = 1;

double F_Hook;
double k = 0.5f; //Hooks Constant
double Fx, Fy, Fz;
double lambda = 2;
double xAttractor = 0; double yAttractor = 0; double thetaA = 0;
double K = 0.0005;

#define DEBUG0 0
#define DEBUG 1


// ************************* CALCULATE DISTANCE FROM SPHERE ******************************* //
void calculateDistanceToSphere(int iter, int rank, double rcvbuffer[16*6], double thrust[3])
{   
    /*
    >   One of the Prime Kinematics Function. Calculates the distance of the UAV to the center of the sphere, 
        and also to one of the points in the concentric circle on the surface of the sphere.
    >   Calculates The required Thrust to go to the set Point.
    >   Calculates Normal Vector to the radial vectors, and applies thrust to generate rotational velocities.
        It does this through using the Dot Product = 0 for normal vector rule.

    >   Input: int current iteration number, int rank number, double rcvbuffer[16*6], double thrust[3].
    >   Output: None (Mutates double thrust[3]).
    */

    // get current UAV (rank) coordinates
    double damp = 1/(iter*10);
    x_0 = rcvbuffer[(6*rank) + 0]; y_0 = rcvbuffer[(6*rank) + 1]; z_0 = rcvbuffer[(6*rank) +2];
    vx0 = rcvbuffer[6 * rank + 3]; vy0 = rcvbuffer[6*rank + 4]; vz0 = rcvbuffer[6*rank + 5];

    // ------------------------------- CENTRAL ATTRACTOR ------------------------------ //
    // These attractors attract the UAVs on different concentric points on the surface of the sphere,
    // to avoid collision. The sphere's thrust towards different points on the sphere surface.
    thetaA = ((double)rank - 1.0) * 24.0;

    xAttractor = 10 * sin(thetaA * (pi/180));
    yAttractor = 10 * cos(thetaA * (pi/180));

    double xDist_A = xAttractor  - x_0;
    double yDist_A = yAttractor  - y_0;
    double zDist_A = sphereCenter - z_0;
    // Distance To Attractor on Surface.
    double dist_A = sqrt( (pow ( xDist_A, 2)) + (pow ( yDist_A, 2)) + (pow ( zDist_A, 2)) ); 


    // -------------------------- CENTER OF SPHERE DISTANCE ----------------------------//
    // Calculates Center of the Sphere, to have a reference for the Hook's Law to attract towards with some
    // displacement.
    double xDist = 0.0f  - x_0;
    double yDist = 0.0f  - y_0;
    double zDist = sphereCenter - z_0;
    double distCenter = sqrt( (pow ( xDist, 2)) + (pow ( yDist, 2)) + (pow ( zDist, 2)) );


    // ---------------- F_norm Component Towards Surface Calculation --------------------------
    // Calculates x,y,z component thrust towards the surface of the Sphere.
    double xFnorm, yFnorm, zFnorm;
    xFnorm = xDist_A/dist_A;
    yFnorm = yDist_A/dist_A;
    zFnorm = zDist_A/dist_A;


    // ----------------------------- Get Velocity Norm ------------------------------------- //
    // Calculates the current velocity Norm. Uses this to calculate normal to the current Norm.
    // Uses this thrust to calculate required Centripetal force for keeping in Orbit.

    double velocityNorm = sqrt( (pow ( vx0, 2)) + (pow ( vy0, 2)) + (pow ( vz0, 2)) );
    double vNormX = vx0/velocityNorm;
    double vNormY = vy0/velocityNorm;
    double vNormZ = vz0/velocityNorm;

    //-------------------------------- HOOK's LAW ------------------------------------//
    F_Hook =  -K * (10 - distCenter); // Hook's Law Implementation.

    if (iter < 250)
    {
        // -------- Comment These for WITHOUT HOOK's LAW --------------- //
        Fx = F_Hook * xFnorm; 
        Fy = F_Hook * yFnorm; 
        Fz = F_Hook * zFnorm;
        
        if ( distCenter <9.5 || distCenter>10.5 )
        {
            //thrust variable is used as Acceleration, since mass = 1kg
            thrust[0] = Fx; // * exp(-lambda * iter); 
            thrust[1] = Fy; // * exp(-lambda * iter); 
            thrust[2] = Fz; // * exp(-lambda * iter); 
        }   
        else
        {   
            // Stop Central Thrusting when the Surface. Activate tangential thrust at this point.
            thrust[0] = 0; // * exp(-lambda * iter); 
            thrust[1] = 0; // * exp(-lambda * iter); 
            thrust[2] = 0; // * exp(-lambda * iter); 
        }
    }
    else
    {

        // ------------------------- xDistance Calculation to the Center ----------------------------- //
        double xFnormCenter, yFnormCenter, zFnormCenter;
        xFnormCenter = xDist/distCenter;
        yFnormCenter = yDist/distCenter;
        zFnormCenter = zDist/distCenter;

        // -------------- Find Various Perpendicular Directions Using Dot Product == 0  --------------- //

        // Find a range of Perpendicular Direction based on the Rank of the UAVs. Uses the principle of 
        // Dot Products = 0 for normal in a slightly different way (just exchanging x's and y's and z's
        // among themselves! )

        double tFx=0; double tFy=0; double tFz=0;

        if (rank%3 == 0){
            tFx = -yFnormCenter; tFy = xFnormCenter; tFz = zFnormCenter; // Tangential Thrust
        }
        else if (rank%2 == 0)
        {
            tFx = -zFnormCenter; tFy = yFnormCenter; tFz = xFnormCenter; // Tangential Thrust
        }
        else if (rank%5 == 0)
        {
            tFx = xFnormCenter; tFy = zFnormCenter; tFz = -yFnormCenter; // Tangential Thrust
        }
        else
        {
            tFx = zFnormCenter; tFy = yFnormCenter; tFz = -xFnormCenter; // Tangential Thrust
        }
        
        Fx = K * tFx; 
        Fy = K * tFy; 
        Fz = K * tFz;

        //---------------------------------- Centripetal Thrust ---------------------------------------//
        double centralThrust = (K*82) * mass * (velocityNorm * velocityNorm) / radius;

        // --------------------- Centripetal Component Thrust Calculation -----------------------------//
        double xCenterThrust = xFnorm * centralThrust;
        double yCenterThrust = yFnorm * centralThrust;
        double zCenterThrust = zFnorm * centralThrust;

        thrust[0] = xCenterThrust + Fx;
        thrust[1] = yCenterThrust + Fy; 
        thrust[2] = zCenterThrust + Fz; 
    }

}



//************************** CALCULATE UAV LOCATION ***************************************

double ax, ay, az;
// double vx0 = 0.0f, vy0 =0.0f,  vz0 = 0.0f;
double maxThrust = 1.0f;

double dt = 0;
double t = 0; double tempT = 0;
void calcualteUAVsLocation(int iter, int rank, double buffer[6], double rcvbuffer[16*6])
{   
    /*
    >   The other Prime Kinematics Function. Calculates the velocity, acceleration and new x,y,z coordinates
        after receiving the new Thrust from the function "calculateDistanceToSphere()".
    >   Input: int current iteration number, int rank number, double buffer[6], double rcvbuffer[16*6].
    >   Output: None (Mutates double rcvbuffer[16*6]).
    */

    // Get Coordinates and Velocities from received buffer
    x_0 = rcvbuffer[6 * rank]; y_0 = rcvbuffer[6*rank + 1]; z_0 = rcvbuffer[6*rank +2];
    vx0 = rcvbuffer[6 * rank + 3]; vy0 = rcvbuffer[6*rank + 4]; vz0 = rcvbuffer[6*rank + 5];

    // ---------------------- Time -----------------------//
    // Temporal Updates to move the simulation forward in time.
    tempT = 1.0/1000.0; //miliseconds
    t = (dt*100.0) * tempT;

    // ========================== Get's Thrust based on distance ========================== //
        calculateDistanceToSphere(iter, rank, rcvbuffer, thrust);

        // Set Directional Thrust 
        thrust[0] = thrust[0] * maxThrust; thrust[1] = thrust[1] * maxThrust; thrust[2] = thrust[2] * maxThrust;

        // Get Thrust
        ax = thrust[0]; ay = thrust[1]; az = thrust[2];

        // Calculate Velocity
        double vx = vx0 + ax * t;
        double vy = vy0 + ay * t;
        double vz = vz0 + az * t;

        // Calculate new x, y, z.
        double x = x_0 + vx0 + 0.5*ax * (t*t);
        double y = y_0 + vy0 + 0.5*ay * (t*t);
        double z = z_0 + vz0 + 0.5*az * (t*t);

        rcvbuffer[6 * rank] = x;
        rcvbuffer[6*rank + 1] = y;
        rcvbuffer[6*rank +2] = z;

        //---------- Fill up receivebuffer and send back ------------//
        rcvbuffer[6 * rank + 3] = vx;
        rcvbuffer[6 * rank + 4] = vy;
        rcvbuffer[6 * rank + 5] = vz;

    //------ Increase Time Step ---------//
    ++dt; //update time step.
}






/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*********************************************************************************************************//
//================================ Some CAMERA Functions for Functionality ================================//
//*********************************************************************************************************//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Reshapes the model after Window Resizing.
void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
              eye_x, eye_y, eye_z,
              center_x, center_y, center_z,
              0.0, 0.0, 1.0);
}

void processNormalKeys(unsigned char key, int xx, int yy)
{
    switch (key) {
        case ESC:
            exit(0);
            break;
        default:
            break;
    }
    if (key == 'u' || key == 'U')  // move camera up case
    {
        pressD = true;
        eyeZ += 5;
        glutPostRedisplay();
    }
    if (key == 'd' || key == 'D') // move camera down case
    {
        pressD = true;
        eyeZ -= 5;
        // deltaMoveZ = -1.0;
        glutPostRedisplay();
    }    
}

// =================== Handles Key Press for Arrow Keys ======================
void pressSpecialKey(int key, int xx, int yy)
{
    switch (key)
    {
        case GLUT_KEY_UP: deltaMove = 1.0; break;
        case GLUT_KEY_DOWN: deltaMove = -1.0; break;
        case GLUT_KEY_RIGHT: deltaMoveH = 1.0; break;
        case GLUT_KEY_LEFT: deltaMoveH = -1.0; break;            
    }
}


// -------------- Handes case for when Key is Released after a press.----------------------
void releaseSpecialKey(int key, int x, int y)
{
    switch (key)
    {
        case GLUT_KEY_UP: deltaMove = 0.0; break;
        case GLUT_KEY_DOWN: deltaMove = 0.0; break;
        case GLUT_KEY_RIGHT: deltaMoveH = 0.0; break;
        case GLUT_KEY_LEFT: deltaMoveH = 0.0; break;
    }
}


// ========================== CAMERA UPDATE FUNCTIONS =======================================
void update(void)
{
    if (deltaMove || deltaMoveH || deltaMoveZ) { // update camera position
        eye_x += deltaMove * lx * 0.7;
        eye_x += deltaMove * ly * 0.7;
        eye_y += deltaMoveH * lx * 0.7;
        eye_y += deltaMoveH * ly * 0.7;
    }
    glutPostRedisplay(); // redisplay everything
    
    if (pressD)
    {
        eye_z += eyeZ;
    }
    glutPostRedisplay(); // redisplay everything
}

void mouseMove(int x, int y)
{
    if (isDragging)
    { // only when dragging
        // update the change in angle
        deltaAngle = (x - xDragStart) * 0.005;
        
        // camera's direction is set to angle + deltaAngle
        lx = -sin(angle + deltaAngle);
        ly = cos(angle + deltaAngle);
    }
}


void mouseButton(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN)
        { // left mouse button pressed
            isDragging = 1; // start dragging
            xDragStart = x; // save x where button first pressed
        }
        else
        { /* (state = GLUT_UP) */
            angle += deltaAngle; // update camera turning angle
            isDragging = 0; // no longer dragging
        }
    }
}

// ======== CLEARS SOME VARIABLES AFTER CAMERA MOVEMENT TO STOP MOVEMENT =========
void clearCameraVariables()
{
    eyeZ = 0;
}

#endif