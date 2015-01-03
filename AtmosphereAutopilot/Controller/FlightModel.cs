using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.IO;

namespace AtmosphereAutopilot
{

	/// <summary>
	/// Class for current vessel flight model calculations
	/// </summary>
	class FlightModel
	{
		/// <summary>
		/// Enumeration of all handeled flight parameters
		/// </summary>
		enum FCharacter
		{
			pitch = 0,		// pitch control value
			roll,			// roll control value
			yaw,			// yaw control value
			aoa_pitch,
			aoa_slide,
			surf_pitch,		// angle between vessel forward and horizont
			surf_roll,		// angle between vessel wings and horizont
			air_speed
		}

		public FlightModel()
		{
			for (int i = 0; i < Param_count; i++)
				telemetry[i] = new CircularBuffer<double>(Steps_remembered, true);
		}

		public const int Param_count = 8;
		public const int Steps_remembered = 10;

		double time = 0.0;				// current time

		public void OnFixedUpdate()
		{
			var state = FlightInputHandler.state;
			telemetry[(int)FCharacter.pitch].Put(state.pitch);
			telemetry[(int)FCharacter.roll].Put(state.roll);
			telemetry[(int)FCharacter.pitch].Put(state.pitch);
		}

		CircularBuffer<double>[] telemetry = new CircularBuffer<double>[Param_count];
	}
}
