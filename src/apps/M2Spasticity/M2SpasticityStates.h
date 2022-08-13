/**
 * /file M2SpasticityStates.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2020-12-09
 *
 * \copyright Copyright (c) 2020
 *
 */

#ifndef M2DemoSTATE_H_DEF
#define M2DemoSTATE_H_DEF

#include <time.h>
#include <iostream>

#include "RobotM2.h"
#include "State.h"

using namespace std;

/**
* declare global variables for each individual

VM2 global_center_point;
VM2 global_start_point;
double global_radius;
double global_start_angle;
*/


/**
 * \brief Conversion from a timespec structure to seconds (double)
 *
 */
double timeval_to_sec(struct timespec *ts);


/**
 * \brief Generic state type for used with M2Spasticity, providing running time and iterations number.
 *
 */
class M2TimedState : public State {
   protected:
    /**
    *  \todo Might be good to make these Const
    *
    */
    RobotM2 *robot;                               /*<!Pointer to state machines robot object*/

    M2TimedState(StateMachine *m, RobotM2 *M2, const char *name = NULL): State(m, name), robot(M2){};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Timing
        clock_gettime(CLOCK_MONOTONIC, &initTime);
        lastTime = timeval_to_sec(&initTime);

        elapsedTime=0;
        iterations=0;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Compute some basic time values
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        double now = timeval_to_sec(&ts);
        elapsedTime = (now-timeval_to_sec(&initTime));
        dt = now - lastTime;
        lastTime = now;

        iterations++;

        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};


   protected:
    struct timespec initTime;   /*<! Time of state init */
    double lastTime;            /*<! Time of last during() call (in seconds since state init())*/
    double elapsedTime;         /*<! Time since state init() in seconds*/
    double dt;                  /*<! Time between last two during() calls (in seconds)*/
    unsigned long int iterations;
};


class M2DemoState : public M2TimedState {

   public:
    M2DemoState(StateMachine *m, RobotM2 *M2, const char *name = "M2 Test State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    VM2 qi, Xi, tau;
};



/**
 * \brief Position calibration of M2. Go to the bottom left stops of robot at constant torque for absolute position calibration.
 *
 */
class M2Calib : public M2TimedState {

   public:
    M2Calib(StateMachine *m, RobotM2 *M2, const char *name = "M2 Calib State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
     int calibAttempts=0;
};



class M2ControlFB: public M2TimedState {

   public:
    M2ControlFB(StateMachine *m, RobotM2 *M2, const char *name = "M2 Control Feedback"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);
    bool isControlDone() {return ControlDone;}

    double Cost=0;
    int Iter=0;
    double TorqueFFST=0;

   private:
    double TorqueFF;
    bool ControlDone=false;
    double startTime;
    bool FFControl=true;
    bool CostCal=true;
    double T;
    double FFUpdate;
    double Dither1;
    double Dither2;
    float k_i=1.; //Integral gain
};





/**
 * \brief Provide end-effector mass compensation on M2. Mass is controllable through keyboard inputs.
 *
 */
class M2Transparent : public M2TimedState {

   public:
    M2Transparent(StateMachine *m, RobotM2 *M2, const char *name = "M2 Transparent"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    Eigen::Matrix2d ForceP;

    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    VM2 X;
    VM2 dX;
    VM2 Fm;
    VM2 Vd;
};


/**
 * \brief End-effector arc circle trajectory (position over velocity)
 *
 */
class M2ArcCircle : public M2TimedState {

   public:
    M2ArcCircle(StateMachine *m, RobotM2 *M2, const char *name = "M2 Arc Circle"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool GoToStartPt() {return goToStartPt;}

   private:
    bool movement_finished;
    bool goToStartPt=false;

    double radius;
    double theta_s;
    double thetaRange;
    double theta;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
    double ang_vel[9] = {10, 20, 30, 40, 50, 60, 70, 80, 90};
};


/**
 * \brief Basic impedance control on a static point.
 *
 */
class M2DemoImpedanceState : public M2TimedState {

   public:
    M2DemoImpedanceState(StateMachine *m, RobotM2 *M2, const char *name = "M2 Demo Impedance State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    VM2 Xi;
    double k = 700;     //! Impedance proportional gain (spring)
    double d = 2;       //! Impedance derivative gain (damper)
    bool init=false;

    unsigned int nb_samples=10000;
    double dts[10000];
    double dX[10000];
    int new_value;
};


/**
 * \brief Point to tpoint position control with min jerk trajectory interpolation
 *
 */
class M2MinJerkPosition: public M2TimedState {

   public:
    M2MinJerkPosition(StateMachine *m, RobotM2 *M2, const char *name = "M2 Demo Minimum Jerk Position"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool nextState() {return nextState_;}

   private:
    bool nextState_=false;
    double startTime;
    VM2 Xi, Xf;
    double T;
    float k_i=1.; //Integral gain
};


/**
 * \brief Movement recording
 *
 */
class M2Recording : public M2TimedState {

   public:
    M2Recording(StateMachine *m, RobotM2 *M2, const char *name = "M2 Recording State"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isRecordingDone() {return recordingDone;}
    bool isRecordingError() {return recordingError;}

   private:
    Eigen::Matrix2d ForceP;

    Eigen::Matrix2d B;
    Eigen::Matrix2d M;
    Eigen::Matrix2d Operator;
    VM2 X;
    VM2 dX;
    VM2 Fm;
    VM2 Vd;

    bool recordingDone=false;
    bool recordingError=false;

    int RecordingPoint;
    static const int MaxRecordingPts = 10100;
    VM2 PositionNow;
    VM2 PositionRecorded[MaxRecordingPts];

    int n;
    VM2 centroid;
    double Mxx, Myy, Mxy, Mxz, Myz, Mzz;
    double Xi, Yi, Zi;
    double Mz, Cov_xy, Mxz2, Myz2;
    double A2, A1, A0, A22;
    double epsilon;
    double ynew, yold, xnew, xold;
    int IterMax;
    double Dy, DET;
    VM2 Center;
    double radius;
    double start_angle;
    VM2 StartPt;
    //VM2 testing;
};


/**
 * \brief Movement testing
 *
 */
class M2CircleTest : public M2TimedState {

   public:
    M2CircleTest(StateMachine *m, RobotM2 *M2, const char *name = "M2 Circle Test"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isTestingDone() {return testingDone;}
    bool isTestingError() {return testingError;}

   private:
    bool testingDone = false;
    bool testingError = false;
    bool movement_finished;
    double radius;
    double theta_s;
    double thetaRange;
    double theta;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
};


/**
 * \brief End-effector arc circle trajectory (position over velocity) back to starting point
 *
 */
class M2ArcCircleReturn : public M2TimedState {

   public:
    M2ArcCircleReturn(StateMachine *m, RobotM2 *M2, const char *name = "M2 Arc Circle Return"):M2TimedState(m, M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

   private:
    bool finished;
    double radius;
    double theta_s;
    double startReturnAngle;
    double thetaReturnRange;
    double thetaReturn;
    int sign;
    double dTheta_t; //Movement target velocity (max of profile) in deg.s-1
    double ddTheta=200; //in deg.s-2
    VM2 centerPt;
    VM2 startingReturnPt;
    double t_init, t_end_accel, t_end_cstt, t_end_decel;
};

#endif
