//==============================================================================
/*
    ME 384 Haptics and Teleoperated Systems
    Rowing Project Teleoperation code
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include <cmath>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// maximum number of devices supported by this application
const int MAX_DEVICES = 16;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight* light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// labels to display each haptic device model
cLabel* labelHapticDeviceModel[MAX_DEVICES];

// labels to display the position [m] of each haptic device
cLabel* labelHapticDevicePosition[MAX_DEVICES];

// global variable to store the position [m] of each haptic device
cVector3d hapticDevicePosition[MAX_DEVICES];

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// some small spheres (cursor) representing position of each haptic device
cShapeSphere* cursor[MAX_DEVICES];

// some lines representing the velocity vector of each haptic device
cShapeLine* velocity[MAX_DEVICES];

//// a small sphere (cursor) representing the haptic device 
//cShapeSphere* cursor[MAX_DEVICES];

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// int for toggling between haptic modes
int hapticMode;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// vector to hold previous position of the cursor
cVector3d prevPosition(1.0, 0.0, 0.0);

//// initialize start time 
//std::chrono::time_point<std::chrono::steady_clock> startTime{};

// create clock
cPrecisionClock stopWatch;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);


//==============================================================================
/*
    TEMPLATE:    application.cpp

    Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Mode 1: Spring Force" << endl;
    cout << "[2] - Mode 2: Damping Field" << endl;
    cout << "[3] - Mode 3: Virtual Wall" << endl;
    cout << "[4] - Mode 4: Virtual Wall with Decaying Sinusoid" << endl;
    cout << "[5] - Mode 5: Bilateral Teleoperation (PD, Position-Exchange)" << endl;
    cout << "[6] - Mode 6: Bilateral Teleoperation (PD, Position-Exchange, Position-Scaled)" << endl;
    cout << "[7] - Mode 7: Bilateral Teleoperation (PD, Position-Exchange, Force-Scaled)" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),    // look at position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // create a font
    font = NEW_CFONT_CONSOLAS_18();


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    numHapticDevices = handler->getNumDevices();

    // setup each haptic device
    for (int i = 0; i < numHapticDevices; i++)
    {
        // get a handle to the first haptic device
        handler->getDevice(hapticDevice[i], i);

        // open a connection to haptic device
        hapticDevice[i]->open();

        // calibrate device (if necessary)
        hapticDevice[i]->calibrate();

        // retrieve information about the current haptic device
        cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

        // create a sphere (cursor) to represent the haptic device
        cursor[i] = new cShapeSphere(0.01);

        // insert cursor inside world
        world->addChild(cursor[i]);

        // create small line to illustrate the velocity of the haptic device
        velocity[i] = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));

        // insert line inside world
        world->addChild(velocity[i]);


        // display a reference frame if haptic device supports orientations
        if (info.m_sensedRotation == true)
        {
            // display reference frame
            cursor[i]->setShowFrame(true);

            // set the size of the reference frame
            cursor[i]->setFrameSize(0.05);
        }

        // if the device has a gripper, enable the gripper to simulate a user switch
        hapticDevice[i]->setEnableGripperUserSwitch(true);

        // create a label to display the haptic device model
        labelHapticDeviceModel[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDeviceModel[i]);
        labelHapticDeviceModel[i]->setText(info.m_modelName);

        // create a label to display the position of haptic device
        labelHapticDevicePosition[i] = new cLabel(font);
        camera->m_frontLayer->addChild(labelHapticDevicePosition[i]);
    }

    cout << "Haptic Devices Detected:" << numHapticDevices << endl;


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONT_CALIBRI_20();

    // create a label to display the haptic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(20, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'q'))
    {
        exit(0);
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }

    if (key == '1')
    {
        hapticMode = 1;
    }

    if (key == '2')
    {
        hapticMode = 2;
    }

    if (key == '3')
    {
        hapticMode = 3;
    }

    if (key == '4')
    {
        hapticMode = 4;
    }

    //if (key == '5')
    //{
    //    hapticMode = 5;
    //}

    //if (key == '6')
    //{
    //    hapticMode = 6;
    //}

    //if (key == '7')
    //{
    //    hapticMode = 7;
    //}

    //if (key == '8')
    //{
    //    hapticMode = 8;
    //}
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    for (int i = 0; i < numHapticDevices; i++)
    {
        hapticDevice[i]->close();
    }

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(20, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (windowW - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);

    // signal frequency counter
    freqCounterGraphics.signal(1);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // main haptic simulation loop
    while (simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // for teleop modes: read states of the devices individually
        cVector3d position1;
        cVector3d velocity1;
        cMatrix3d rotation1;
        cVector3d rotVel1;
        hapticDevice[0]->getPosition(position1);
        hapticDevice[0]->getLinearVelocity(velocity1);
        hapticDevice[0]->getRotation(rotation1);
        hapticDevice[0]->getAngularVelocity(rotVel1);


        cVector3d position2;
        cVector3d velocity2;
        cMatrix3d rotation2;
        cVector3d rotVel2;
        hapticDevice[1]->getPosition(position2);
        hapticDevice[1]->getLinearVelocity(velocity2);
        hapticDevice[1]->getRotation(rotation2);
        hapticDevice[1]->getAngularVelocity(rotVel2);


        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // update position and orienation of cursor
        cursor[0]->setLocalPos(position1);
        cursor[0]->setLocalRot(rotation1);

        cursor[1]->setLocalPos(position2);
        cursor[1]->setLocalRot(rotation2);

        /////////////////////////////////////////////////////////////////////
        // COMPUTE FORCES
        /////////////////////////////////////////////////////////////////////

        // variables for forces
        cVector3d force(0, 0, 0);
        cVector3d torque(0, 0, 0);
        double gripperForce = 0.0;

        // force variables for teleoperation
        cVector3d force1(0, 0, 0);
        cVector3d force2(0, 0, 0);
        cVector3d torque1(0, 0, 0);
        cVector3d torque2(0, 0, 0);

        // Hold Lambda in zero position on startup and on pressing the 1 key
        if (hapticMode == 1) {
            // define constants
            double K = 200;
            double b = 8;
            double Kr = 0.5;
            double br = 0.01;

            // zero position of the lambda
            cVector3d zeroPos(0, 0, 0); // define target location 
            cMatrix3d startRot1(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            cMatrix3d startRot2(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            //cMatrix3d startRot;
            //startRot.identity(); 

            // compute spring force 
            cVector3d springForce1 = -K * (position1 - zeroPos) - b * velocity1;
            cVector3d springForce2 = -K * (position2 - zeroPos) - b * velocity2;
            force1.add(springForce1);
            force2.add(springForce2);

            // compute spring torque
            cVector3d axis1;
            cVector3d axis2;
            double angle1;
            double angle2;
            cMatrix3d deltaRotation1 = cTranspose(startRot1) * rotation1; // NO transpose! So that it flips
            cMatrix3d deltaRotation2 = cTranspose(startRot2) * rotation2;
            deltaRotation1.toAxisAngle(axis1, angle1);
            deltaRotation2.toAxisAngle(axis2, angle2);
            torque1 = -(rotation1 * ((Kr * angle1) * axis1)) - br * (rotVel1);
            torque2 = -(rotation2 * ((Kr * angle2) * axis2)) - br * (rotVel2);

        }

        // Null condition - no leader-follower haptics
        if (hapticMode == 2) {
            // define constants
            double K = 200;
            double b = 8;
            double Kr = 0.5;
            double br = 0.01;

            // zero position of the lambda
            cVector3d zeroPos(0, 0, 0); // define target location 

            // compute spring force 
            cVector3d springForce1 = -K * (position1 - zeroPos) - b * velocity1;
            cVector3d springForce2 = -K * (position2 - zeroPos) - b * velocity2;
            force1.add(springForce1);
            force2.add(springForce2);

        }

        // Unilateral teleoperation 
        if (hapticMode == 3) {
            // rotational spring constants
            double Kr1 = 10.0; // I was using 2
            double Kr2 = 10.0;

            // derivative spring constants
            double Krd1 = 0.01; // N.m
            double Krd2 = 0.01;

            // compute linear forces fixing the handle in a 'pivot point'
            cVector3d zeroPos(0, 0, 0); // define target location 

            // define constants attracting cursor to the pivot point
            double K = 200;
            double b = 4;

            // compute spring force
            cVector3d springForce1 = -K * (position1 - zeroPos) - b * velocity1;
            cVector3d springForce2 = -K * (position2 - zeroPos) - b * velocity2;
            force1.add(springForce1);
            force2.add(springForce2);

            // extract roll angles
            double roll1 = atan2(rotation1(2, 1), rotation1(2, 2));
            double roll2 = atan2(rotation2(2, 1), rotation2(2, 2));
            double pitch1 = atan2(-rotation1(2, 0), sqrt(rotation1(2, 1) * rotation1(2, 1) + rotation1(2, 2) * rotation1(2, 2)));
            double pitch2 = atan2(-rotation2(2, 0), sqrt(rotation2(2, 1) * rotation2(2, 1) + rotation2(2, 2) * rotation2(2, 2)));
            // yaw of each device (needed to locate each device's pitch axis)
            double yaw1 = atan2(rotation1(1, 0), rotation1(0, 0));
            double yaw2 = atan2(rotation2(1, 0), rotation2(0, 0));

            // current pitch-joint axis of each device, in the base frame
            cVector3d pitchAxis1(-sin(yaw1), cos(yaw1), 0.0);
            cVector3d pitchAxis2(-sin(yaw2), cos(yaw2), 0.0);

            // mirrored error: zero when roll2 = -roll1
            double rollErr = roll1 + roll2;
            double pitchErr = pitch2 - pitch1;
            rollErr = atan2(sin(rollErr), cos(rollErr));   // wrap to [-pi, pi]

            // mirrored rate error: zero when w2 = -w1
            double rollRateErr = rotVel1.x() + rotVel2.x();
            double pitchRateErr = rotVel2.dot(pitchAxis2) - rotVel1.dot(pitchAxis1);

            //// axial torque on device 2 only
            //torque2 = cVector3d(0.0, -Kr2 * pitchErr - Krd2 * pitchRateErr, 0.0);

            // torque about the follower's own pitch axis
            double tau = -Kr2 * pitchErr - Krd2 * pitchRateErr;
            torque2 = tau * pitchAxis2;

        }

        // Bilateral teleoperation 
        if (hapticMode == 4) {
            // rotational spring constants
            double Kr1 = 10.0;
            double Kr2 = 10.0;

            // derivative spring constants
            double Krd1 = 0.01; // N.m
            double Krd2 = 0.01;

            // compute linear forces fixing the handle in a 'pivot point'
            cVector3d target(0, 0, 0); // define target location 

            // define spring constant attracting cursor to the target
            double K = 200;
            double b = 4;

            // compute spring force
            cVector3d springForce1 = -K * (position1 - target) - b * velocity1;
            cVector3d springForce2 = -K * (position2 - target) - b * velocity2;
            force1.add(springForce1);
            force2.add(springForce2);

            // extract roll angles
            double roll1 = atan2(rotation1(2, 1), rotation1(2, 2));
            double roll2 = atan2(rotation2(2, 1), rotation2(2, 2));
            double pitch1 = atan2(-rotation1(2, 0), sqrt(rotation1(2, 1) * rotation1(2, 1) + rotation1(2, 2) * rotation1(2, 2)));
            double pitch2 = atan2(-rotation2(2, 0), sqrt(rotation2(2, 1) * rotation2(2, 1) + rotation2(2, 2) * rotation2(2, 2)));
            // yaw of each device (needed to locate each device's pitch axis)
            double yaw1 = atan2(rotation1(1, 0), rotation1(0, 0));
            double yaw2 = atan2(rotation2(1, 0), rotation2(0, 0));

            // current pitch-joint axis of each device, in the base frame
            cVector3d pitchAxis1(-sin(yaw1), cos(yaw1), 0.0);
            cVector3d pitchAxis2(-sin(yaw2), cos(yaw2), 0.0);

            // mirrored error: zero when roll2 = -roll1
            double rollErr = roll1 + roll2;
            double pitchErr = pitch2 - pitch1;
            rollErr = atan2(sin(rollErr), cos(rollErr));   // wrap to [-pi, pi]

            // mirrored rate error: zero when w2 = -w1
            double rollRateErr = rotVel1.x() + rotVel2.x();
            double pitchRateErr = rotVel2.dot(pitchAxis2) - rotVel1.dot(pitchAxis1);

            //// axial torque on device 2 only
            //torque2 = cVector3d(0.0, -Kr2 * pitchErr - Krd2 * pitchRateErr, 0.0);

            // torque about the follower's own pitch axis
            double tau1 = -Kr1 * pitchErr - Krd1 * pitchRateErr;
            torque1 = -tau1 * pitchAxis1;
            double tau2 = -Kr2 * pitchErr - Krd2 * pitchRateErr;
            torque2 = tau2 * pitchAxis2;

        }


        /////////////////////////////////////////////////////////////////////
        // APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // send computed force, torque, and gripper force to haptic device
        hapticDevice[0]->setForceAndTorqueAndGripperForce(force1, torque1, gripperForce);
        hapticDevice[1]->setForceAndTorqueAndGripperForce(force2, torque2, gripperForce);

        // update frequency counter
        freqCounterHaptics.signal(1);

    }

    // exit haptics thread
    simulationFinished = true;
}

//-----------------------------------------------------------------------------