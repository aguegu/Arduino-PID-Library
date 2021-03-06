#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

#include "Arduino.h"

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0

#define DIRECT  0
#define REVERSE  1

class PID
{
public:

	//commonly used functions **************************************************************************
	PID(double* pInput, double* pOutput, double* pSetpoint, double kp,
			double ki, double kd, int controller_direction);

	void setMode(int Mode);
	// * sets PID to either Manual (0) or Auto (non-0)

	void compute();
	// * performs the PID calculation.  it should be
	//   called every time loop() cycles. ON/OFF and
	//   calculation frequency can be set using SetMode
	//   SetSampleTime respectively

	void setOutputLimits(double, double);
	//clamps the output to a specific range. 0-255 by default, but
	//it's likely the user will want to change this depending on
	//the application

	//available but not commonly used functions ********************************************************
	void setTunings(double, double, double);
	// * While most users will set the tunings once in the
	//   constructor, this function gives the user the option
	//   of changing tunings during runtime for Adaptive control

	void setControllerDirection(int);
	// * Sets the Direction, or "Action" of the controller.
	//	 DIRECT means the output will increase when error is positive.
	//	 REVERSE means the opposite.  it's very unlikely that this will be needed
	//   once it is set in the constructor.

	void setSampleTime(int);
	// * sets the frequency, in Milliseconds, with which
	//   the PID calculation is performed.  default is 100

	//Display functions ****************************************************************
	double getKp(); // These functions query the pid for interal values.
	double getKi(); //  they were created mainly for the pid front-end,
	double getKd(); // where it's important to know what is actually

	int getMode(); //  inside the PID.
	int getDirection(); //

private:
	void initialize();

	double _kp_disp; // * we'll hold on to the tuning parameters in user-entered
	double _ki_disp; //   format for display purposes
	double _kd_disp; //

	double _kp; // * (P)roportional Tuning Parameter
	double _ki; // * (I)ntegral Tuning Parameter
	double _kd; // * (D)erivative Tuning Parameter

	int _controller_direction;

	double *_pInput; // * Pointers to the Input, Output, and Setpoint variables
	double *_pOutput; //   This creates a hard link between the variables and the
	double *_pSetpoint; //   PID, freeing the user from having to constantly tell us
						//   what these values are.  with pointers we'll just know.

	unsigned long _last_time;
	double _i_term, _last_input;

	unsigned long _sample_time;
	double _out_min, _out_max;
	bool _inAuto;
};
#endif

