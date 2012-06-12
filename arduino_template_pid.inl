/** @file
	@brief Template Implementations - included from within arduino_template_pid.h

	@date 2008-2012

	@author
	Brett Beauregard
	<br3ttb@gmail.com>
	http://brettbeauregard.com

	@author
	Ryan Pavlik
	<rpavlik@iastate.edu> and <abiryan@ryand.net>
	http://academic.cleardefinition.com/
	Iowa State University Virtual Reality Applications Center
	Human-Computer Interaction Graduate Program
*/

/*
	Copyright 2008-2011 Brett Beauregard
	Copyright Iowa State University 2012.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, version 3.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef IN_ARDUINO_TEMPLATE_PID_H
# error "Do not include arduino_template_pid.inl directly! It is include from within the .h file"
#endif

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
template<typename T, typename TuningT>
inline GenericPID<T, TuningT>::GenericPID(GenericPID<T, TuningT>::value_type & Input,
        GenericPID<T, TuningT>::value_type & Output,
        GenericPID<T, TuningT>::value_type & Setpoint,
        GenericPID<T, TuningT>::tuning_value_type Kp,
        GenericPID<T, TuningT>::tuning_value_type Ki,
        GenericPID<T, TuningT>::tuning_value_type Kd,
        GenericPID<T, TuningT>::PIDDirection ControllerDirection)
	: controllerDirection(ControllerDirection)
	, myInput(&Input)
	, myOutput(&Output)
	, mySetpoint(&Setpoint)
	, lastTime(millis() - SampleTime)
	, SampleTime(100)
	, outMin(0)		///< default output limit corresponds to the arduino pwm limits
	, outMax(255)	///< default output limit corresponds to the arduino pwm limits
	, inAuto(false)
	, justCalced(false) {
	SetTunings(Kp, Ki, Kd);
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::Compute() {

	justCalced = false;
	if (!inAuto) {
		return;
	}
	timestamp_type now = millis();
	timestamp_type timeChange = (now - lastTime);
	if (timeChange >= SampleTime) {
		/*Compute all the working error variables*/
		value_type input = *myInput;
		value_type error = *mySetpoint - input;
		ITerm += (ki * error);
		if (ITerm > outMax) {
			ITerm = outMax;
		} else if (ITerm < outMin) {
			ITerm = outMin;
		}
		value_type dInput = (input - lastInput);

		/*Compute PID Output*/
		value_type output = static_cast<value_type>(kp * error + ITerm - kd * dInput);
		applyOutputLimit(output);
		*myOutput = output;

		/*Remember some variables for next time*/
		lastInput = input;
		lastTime = now;
		justCalced = true;
	}
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::SetTunings(GenericPID<T, TuningT>::tuning_value_type Kp,
        GenericPID<T, TuningT>::tuning_value_type Ki,
        GenericPID<T, TuningT>::tuning_value_type Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0) {
		return;
	}

	dispKp = Kp;
	dispKi = Ki;
	dispKd = Kd;

	tuning_value_type SampleTimeInSec = static_cast<tuning_value_type>(SampleTime) / 1000;
	kp = controllerDirection * Kp;
	ki = controllerDirection * Ki * SampleTimeInSec;
	kd = controllerDirection * Kd / SampleTimeInSec;

}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::SetSampleTime(GenericPID<T, TuningT>::sample_time_type NewSampleTime) {
	if (NewSampleTime > 0) {
		tuning_value_type ratio = static_cast<tuning_value_type>(NewSampleTime)
		                          / static_cast<tuning_value_type>(SampleTime);
		ki *= ratio;
		kd /= ratio;
		SampleTime = NewSampleTime;
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
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::SetOutputLimits(GenericPID<T, TuningT>::value_type Min,
        GenericPID<T, TuningT>::value_type Max) {
	if (Min >= Max) {
		return;
	}
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		applyOutputLimit(*myOutput);
		applyOutputLimit(ITerm);
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::SetMode(GenericPID<T, TuningT>::PIDMode Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::Initialize() {
	ITerm = *myOutput;
	lastInput = *myInput;
	applyOutputLimit(ITerm);
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
template<typename T, typename TuningT>
inline void GenericPID<T, TuningT>::SetControllerDirection(GenericPID<T, TuningT>::PIDDirection Direction) {
	if (inAuto && Direction != controllerDirection) {
		kp = - kp;
		ki = - ki;
		kd = - kd;
	}
	controllerDirection = Direction;
}


