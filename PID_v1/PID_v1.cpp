/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "PID_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* pInput, double* pOutput, double* pSetpoint, double kp,
		double ki, double kd, int controller_direction)
{
	setOutputLimits(0, 255); //default output limit corresponds to
							 //the arduino pwm limits
	_sample_time = 100; //default Controller Sample Time is 0.1 seconds

	setControllerDirection(controller_direction);
	setTunings(kp, ki, kd);

	_last_time = millis() - _sample_time;
	_inAuto = false;
	_pOutput = pOutput;
	_pInput = pInput;
	_pSetpoint = pSetpoint;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
void PID::compute()
{
	if (!_inAuto)
		return;
	unsigned long now = millis();
	unsigned long time_change = (now - _last_time);
	if (time_change >= _sample_time)
	{
		/*Compute all the working error variables*/
		double input = *_pInput;
		double error = *_pSetpoint - input;
		_i_term += (_ki * error);
		if (_i_term > _out_max)
			_i_term = _out_max;
		else if (_i_term < _out_min)
			_i_term = _out_min;
		double dInput = (input - _last_input);

		/*Compute PID Output*/
		double output = _kp * error + _i_term - _kd * dInput;

		if (output > _out_max)
			output = _out_max;
		else if (output < _out_min)
			output = _out_min;
		*_pOutput = output;

		/*Remember some variables for next time*/
		_last_input = input;
		_last_time = now;
	}
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::setTunings(double kp, double ki, double kd)
{
	if (kp < 0 || ki < 0 || kd < 0) return;

	_kp_disp = kp;
	_ki_disp = ki;
	_kd_disp = kd;

	double sample_time_in_second = ((double) _sample_time) / 1000;
	_kp = kp;
	_ki = ki * sample_time_in_second;
	_kd = kd / sample_time_in_second;

	if (_controller_direction == REVERSE)
	{
		_kp = (0 - kp);
		_ki = (0 - ki);
		_kd = (0 - kd);
	}
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::setSampleTime(int sample_time)
{
	if (sample_time > 0)
	{
		double ratio = (double) sample_time / (double) _sample_time;
		_ki *= ratio;
		_kd /= ratio;
		sample_time = (unsigned long) sample_time;
	}
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::setOutputLimits(double min, double max)
{
	if (min >= max)	return;

	_out_min = min;
	_out_max = max;

	if (_inAuto)
	{
		if (*_pOutput > _out_max)
			*_pOutput = _out_max;
		else if (*_pOutput < _out_min)
			*_pOutput = _out_min;

		if (_i_term > _out_max)
			_i_term = _out_max;
		else if (_i_term < _out_min)
			_i_term = _out_min;
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::setMode(int mode)
{
	bool newAuto = (mode == AUTOMATIC);
	if (newAuto == !_inAuto)
	{ /*we just went from manual to auto*/
		initialize();
	}
	_inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::initialize()
{
	_i_term = *_pOutput;
	_last_input = *_pInput;
	if (_i_term > _out_max)
		_i_term = _out_max;
	else if (_i_term < _out_min)
		_i_term = _out_min;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::setControllerDirection(int direction)
{
	if (_inAuto && direction != _controller_direction)
	{
		_kp = (0 - _kp);
		_ki = (0 - _ki);
		_kd = (0 - _kd);
	}
	_controller_direction = direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::getKp()
{
	return _kp_disp;
}
double PID::getKi()
{
	return _ki_disp;
}
double PID::getKd()
{
	return _kd_disp;
}
int PID::getMode()
{
	return _inAuto ? AUTOMATIC : MANUAL;
}
int PID::getDirection()
{
	return _controller_direction;
}

