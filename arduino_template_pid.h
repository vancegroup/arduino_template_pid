/** @file
	@brief Header

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
	Copyright Brett Beauregard 2008-2011.
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

#ifndef INCLUDED_arduino_template_pid_h_GUID_8703f8c8_49cd_4687_98bd_2fbd30dbdf78
#define INCLUDED_arduino_template_pid_h_GUID_8703f8c8_49cd_4687_98bd_2fbd30dbdf78

// Internal Includes
// - none

// Library/third-party includes
// - none

// Standard includes

#ifdef ARDUINO
# if ARDUINO >= 100
#  include "Arduino.h"
# else
#  include "WProgram.h"
# endif
#else
# error "Not yet ported to this platform."
#endif

template<typename T = double, typename TuningT = double>
class GenericPID {


	public:
		typedef T value_type;
		typedef TuningT tuning_value_type;
		typedef unsigned int sample_time_type;
		typedef unsigned long timestamp_type;

		//Constants used in some of the functions below

		enum PIDMode {
			AUTOMATIC = 1,
			MANUAL = 0
		};
		enum PIDDirection {
			DIRECT = 1,
			REVERSE = -1
		};
		//commonly used functions **************************************************************************
		GenericPID(value_type & Input, value_type & Output, value_type & Setpoint,        // * constructor.  links the PID to the Input, Output, and
		           tuning_value_type Kp, tuning_value_type Ki, tuning_value_type Kd, PIDDirection Direction = DIRECT);     //   Setpoint.  Initial tuning parameters are also set here

		void SetMode(PIDMode Mode);               // * sets PID to either Manual (0) or Auto (non-0)

		void Compute();                       // * performs the PID calculation.  it should be
		//   called every time loop() cycles. ON/OFF and
		//   calculation frequency can be set using SetMode
		//   SetSampleTime respectively

		void SetOutputLimits(value_type Min, value_type Max); //clamps the output to a specific range. 0-255 by default, but
		//it's likely the user will want to change this depending on
		//the application



		//available but not commonly used functions ********************************************************
		void SetTunings(tuning_value_type Kp, tuning_value_type Ki,       // * While most users will set the tunings once in the
		                tuning_value_type Kd);         	  //   constructor, this function gives the user the option
		//   of changing tunings during runtime for Adaptive control
		void SetControllerDirection(PIDDirection);	  // * Sets the Direction, or "Action" of the controller. DIRECT
		//   means the output will increase when error is positive. REVERSE
		//   means the opposite.  it's very unlikely that this will be needed
		//   once it is set in the constructor.
		void SetSampleTime(sample_time_type NewSampleTime);              // * sets the frequency, in Milliseconds, with which
		//   the PID calculation is performed.  default is 100



		//Display functions ****************************************************************
		/* Status Funcions*************************************************************
		* Just because you set the Kp=-1 doesn't mean it actually happened.  these
		* functions query the internal state of the PID.  they're here for display
		* purposes.  this are the functions the PID Front-end uses for example
		******************************************************************************/

		tuning_value_type GetKp() const {
			return dispKp;    // These functions query the pid for interal values.
		}
		tuning_value_type GetKi() const {
			return dispKi;    //  they were created mainly for the pid front-end,
		}
		tuning_value_type GetKd() const {
			return dispKd;    // where it's important to know what is actually
		}
		PIDMode GetMode() const {
			return  inAuto ? AUTOMATIC : MANUAL;   //  inside the PID.
		}
		PIDDirection GetDirection() const {
			return controllerDirection;   //
		}
		bool JustCalculated() const {
			return justCalced;
		}

	private:
		void Initialize();

		template<typename U>
		U getClampedToOutputLimit(U val) {
			U ret;
			if (val > outMax) {
				ret = outMax;
			} else if (val < outMin) {
				ret = outMin;
			} else {
				ret = val;
			}
			return ret;
		}
		template<typename U>
		void applyOutputLimit(U & val) {
			val = getClampedToOutputLimit(val);
		}

		tuning_value_type dispKp;				// * we'll hold on to the tuning parameters in user-entered
		tuning_value_type dispKi;				//   format for display purposes
		tuning_value_type dispKd;				//

		tuning_value_type kp;                  // * (P)roportional Tuning Parameter
		tuning_value_type ki;                  // * (I)ntegral Tuning Parameter
		tuning_value_type kd;                  // * (D)erivative Tuning Parameter

		PIDDirection controllerDirection;

		value_type & myInput;              // * Pointers to the Input, Output, and Setpoint variables
		value_type & myOutput;             //   This creates a hard link between the variables and the
		value_type & mySetpoint;           //   PID, freeing the user from having to constantly tell us
		//   what these values are.  with pointers we'll just know.

		timestamp_type lastTime;
		tuning_value_type ITerm;
		value_type lastInput;

		sample_time_type SampleTime;
		value_type outMin, outMax;
		bool inAuto;
		bool justCalced;			// * flag gets set for one cycle after the pid calculates
};

typedef GenericPID<double> PIDd;
typedef GenericPID<int> PIDi;

#define IN_ARDUINO_TEMPLATE_PID_H
#include <arduino_template_pid.inl>
#undef IN_ARDUINO_TEMPLATE_PID_H


#endif // INCLUDED_arduino_template_pid_h_GUID_8703f8c8_49cd_4687_98bd_2fbd30dbdf78
