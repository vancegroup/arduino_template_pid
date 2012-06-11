/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
template<typename T, typename TuningT>
inline PID<T, TuningT>::PID(PID<T, TuningT>::value_type & Input, PID<T, TuningT>::value_type & Output, PID<T, TuningT>::value_type & Setpoint,
                            PID<T, TuningT>::tuning_value_type Kp, PID<T, TuningT>::tuning_value_type Ki, PID<T, TuningT>::tuning_value_type Kd, PID<T, TuningT>::PIDDirection ControllerDirection)
	: myInput(&Input)
	, myOutput(&Output)
	, mySetpoint(&Setpoint)
	, lastTime(millis() - SampleTime)
	, SampleTime(100)
	, inAuto(false)
	, justCalced(false) {
	PID::SetOutputLimits(0, 255);				//default output limit corresponds to
	//the arduino pwm limits

	PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd);

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/
template<typename T, typename TuningT>
inline void PID<T, TuningT>::Compute() {

	justCalced=false;
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
		justCalced=true;
	}
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
template<typename T, typename TuningT>
inline void PID<T, TuningT>::SetTunings(TuningT Kp, TuningT Ki, TuningT Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0) {
		return;
	}

	dispKp = Kp;
	dispKi = Ki;
	dispKd = Kd;

	tuning_value_type SampleTimeInSec = static_cast<tuning_value_type>(SampleTime) / 1000;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;

	if (controllerDirection == REVERSE) {
		kp = - kp;
		ki = - ki;
		kd = - kd;
	}
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
template<typename T, typename TuningT>
inline void PID<T, TuningT>::SetSampleTime(PID<T, TuningT>::sample_time_type NewSampleTime) {
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
inline void PID<T, TuningT>::SetOutputLimits(T Min, T Max) {
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
inline void PID<T, TuningT>::SetMode(PID<T, TuningT>::PIDMode Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		PID::Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
template<typename T, typename TuningT>
inline void PID<T, TuningT>::Initialize() {
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
inline void PID<T, TuningT>::SetControllerDirection(PID<T, TuningT>::PIDDirection Direction) {
	if (inAuto && Direction != controllerDirection) {
		kp = - kp;
		ki = - ki;
		kd = - kd;
	}
	controllerDirection = Direction;
}


