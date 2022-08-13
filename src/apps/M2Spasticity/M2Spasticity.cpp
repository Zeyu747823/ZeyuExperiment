#include "M2Spasticity.h"

#define OWNER ((M2Spasticity *)owner)

M2Spasticity::M2Spasticity() {
    robot = new RobotM2();
    STest = new SpasticityTest();

    // Create PRE-DESIGNED State Machine events and state objects.
    calibState = new M2Calib(this, robot);
    standbyState = new M2Transparent(this, robot);
    fbcontrol = new M2ControlFB(this, robot);
    recordingState = new M2Recording(this, robot);
    experimentState = new M2ArcCircle(this, robot);
    minJerkState = new M2MinJerkPosition(this, robot);
    experimentReturnState = new M2ArcCircleReturn(this, robot);
    testingState = new M2CircleTest(this, robot);

    endCalib = new EndCalib(this);
    endControl = new EndControl(this);
    goToNextState = new GoToNextState(this);
    goToPrevState = new GoToPrevState(this);
    startRecording = new StartRecording(this);
    endRecording = new EndRecording(this);
    failRecording = new FailRecording(this);
    startTesting = new StartTesting(this);
    endTesting = new EndTesting(this);
    failTesting = new FailTesting(this);
    startTrial = new StartTrial(this);
    startReturn = new StartReturn(this);
    goToTransparent = new GoToTransparent(this);
    maxForceReturn = new MaxForceReturn(this);

    /**
     * \brief add a tranisition object to the arch list of the first state in the NewTransition MACRO.
     * Effectively creating a statemachine transition from State A to B in the event of event c.
     * NewTranstion(State A,Event c, State B)
     *
     */
     NewTransition(calibState, endCalib, standbyState);
     NewTransition(standbyState, goToNextState, minJerkState);
     NewTransition(minJerkState, goToNextState, fbcontrol);
     NewTransition(fbcontrol, endControl, minJerkState);

    //Initialize the state machine with first state of the designed state machine, using baseclass function.
    StateMachine::initialize(calibState);
}
M2Spasticity::~M2Spasticity() {
    delete robot;
}

/**
 * \brief start function for running any designed statemachine specific functions
 * for example initialising robot objects.
 *
 */
void M2Spasticity::init() {
    spdlog::debug("M2Spasticity::init()");
    if(robot->initialise()) {
        initialised = true;
        logHelper.initLogger("M2SpasticityLog", "logs/M2_ILC.csv", LogFormat::CSV, true);
        logHelper.add(time_running, "Time (s)");
        logHelper.add(robot->getEndEffPositionRef(), "Position");
        logHelper.add(robot->getEndEffVelocityRef(), "Velocity");
        logHelper.add(robot->getEndEffForceRef(), "Force");
        logHelper.add(RecordState, "State");
        logHelper.startLogger();
    }
    else {
        initialised = false;
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM); //Clean exit
    }
    running = true;
    time_init = std::chrono::steady_clock::now();
    time_running = 0;
}

void M2Spasticity::end() {
    if(initialised) {
        if(logHelper.isStarted())
            logHelper.endLog();
        currentState->exit();
        robot->disable();
    }
}

////////////////////////////////////////////////////////////////
// Events ------------------------------------------------------
///////////////////////////////////////////////////////////////

/**
 * \brief Statemachine to hardware interface method. Run any hardware update methods
 * that need to run every program loop update cycle.
 *
 */
void M2Spasticity::hwStateUpdate(void) {
    time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - time_init).count()) / 1e6;
    robot->updateRobot();
}



bool M2Spasticity::EndCalib::check() {
    return OWNER->calibState->isCalibDone();
}
bool M2Spasticity::EndControl::check() {
    return OWNER->fbcontrol->isControlDone();
}


bool M2Spasticity::GoToNextState::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==1 || OWNER->minJerkState->nextState()) )
        return true;

    //Otherwise false
    return false;
}


bool M2Spasticity::GoToPrevState::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==2) )
        return true;

    //Otherwise false
    return false;
}


bool M2Spasticity::StartRecording::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==3) )
        return true;

    //Otherwise false
    return false;
}


bool M2Spasticity::EndRecording::check() {
    return OWNER->recordingState->isRecordingDone();
}


bool M2Spasticity::FailRecording::check() {
    return OWNER->recordingState->isRecordingError();
}


bool M2Spasticity::StartTesting::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==4) )
        return true;

    //Otherwise false
    return false;
}


bool M2Spasticity::EndTesting::check() {
    return OWNER->testingState->isTestingDone();
}


bool M2Spasticity::FailTesting::check() {
    return OWNER->testingState->isTestingError();
}


bool M2Spasticity::StartTrial::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==5) )
        return true;

    //Otherwise false
    return false;
}



bool M2Spasticity::StartReturn::check() {
    return OWNER->experimentState->GoToStartPt();
}



bool M2Spasticity::MaxForceReturn::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==8) )
        return true;

    //Otherwise false
    return false;
}


bool M2Spasticity::GoToTransparent::check() {
    //keyboard or joystick press
    if ( (OWNER->robot->joystick->isButtonPressed(1) || OWNER->robot->keyboard->getNb()==9))
        return true;

    if (OWNER->goToTransparentFlag)
    {
        OWNER->goToTransparentFlag = false;
        return true;
    }

    //Otherwise false
    return false;
}



bool M2Spasticity::configureMasterPDOs() {
    spdlog::debug("M2Spasticity::configureMasterPDOs()");
    return robot->configureMasterPDOs();
}
