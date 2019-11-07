package frc.robot;

/**
 * This class contains static members that define properties of the robot,
 * device IDs, or various unchanging values to be used throughout the robot
 * program.
 */
public class Constants {
	// Prevent initialization from outside
	private Constants() {
	}

	/**
	 * Defines all the CAN IDs for the robot (device IDs for all relevant devices on
	 * the CAN chain).
	 */
	public class CANIDs {
		private CANIDs() {
		}

		public static final int DRIVE_R1_PORT = 2;
		public static final int DRIVE_R2_PORT = 22;
		public static final int DRIVE_L1_PORT = 1;
		public static final int DRIVE_L2_PORT = 21;

		public static final int ELEVATOR_M1 = 12;
		public static final int ELEVATOR_M2 = 11;
		public static final int ELEVATOR_M3 = 10;

		public static final int INTAKE = 4;

		/** TODO: Check */
		public static final int LEFT_WINCH = 30; // Based on orientation where the battery is the front
		public static final int RIGHT_WINCH = 31; // See above
		public static final int VACUUM_PORT = 7;
	}

	public class DigitalAnalogIDs {
		private DigitalAnalogIDs() {
		}

		public static final int PRESSURE_SENSOR_PORT = 0;
	}

	/**
	 * Defines all the solenoid IDs for the PCM (Pneumatics Control Module).
	 */
	public class PCMIDs {
		private PCMIDs() {
		}

		public static final int INTAKE_HEIGHT = 0;
		public static final int CLIMB_RELEASE = 1;
		public static final int ELEVATOR_SHIFT = 2;
		public static final int HATCH_FINGERS = 3;
		public static final int HATCH_EJECTORS = 4;
		public static final int DRIVE_GEAR_SHIFT = 6;
	}

	public static final boolean IS_LEFT_INVERTED = true;
	public static final boolean IS_RIGHT_INVERTED = false;
	public static final boolean IS_DRIVEBASE_BACKWARDS = true;
	public static final double TURN_SENSITIVITY = 1;
	public static final double RADIAL_TURN_SENSITIVITY = 20;
	public static final double SCALING_POWER = 2.75;
	public static final double RAMP_RATE = 0.25; // Seconds to go from 0 to full throttle

	/** Elevator Constants */
	public static final double BOTTOM_LIMIT_POSITION = 0.0; // In rotations by default but using conversion factors we
															// can make this into inches/meters
	public static final double TOP_LIMIT_POSITION = 46.2; // Tested by James 11/5/19 // 172.0 is for low gear // Tested
															// by Ingi 10/31/19
	public static final double TOP_LOW_GEAR_LIMIT_POSITION = 172.0; // Tested by Ingi 10/31/19
	public static final double ELEVATOR_ROTATION_TOLERANCE_LOW_GEAR = 10.0; // Rotations away from the ends of the
																			// elevator to
	// slow down in low gear.
	public static final double ELEVATOR_ROTATION_TOLERANCE_HIGH_GEAR = 3.0; // See above, high gear
	public static final double ROTATION_TOLERANCE_MULTIPLIER = 0.5; // TBD, Placeholder
	public static final double HIGH_GEAR_TO_LOW_GEAR_ROTATIONS = TOP_LOW_GEAR_LIMIT_POSITION / TOP_LIMIT_POSITION; // Multiply
																													// high
																													// gear
																													// position
																													// by
																													// this
																													// to
																													// convert
																													// to
																													// the
																													// equivalent
																													// low
																													// gear
																													// position
	public static final double LOW_GEAR_TO_HIGH_GEAR_ROATIONS = TOP_LIMIT_POSITION / TOP_LOW_GEAR_LIMIT_POSITION; // Multiply
																													// low
																													// gear
																													// position
																													// by
																													// this
																													// to
																													// convert
																													// to
																													// the
																													// equivalent
																													// high
																													// gear
																													// position

	// Constants for high gear/scoring
	public static final double ELEVATOR_P = 0.075;
	public static final double ELEVATOR_I = 0.0;
	public static final double ELEVATOR_D = 0.0;
	public static final double ELEVATOR_F = 0.0;
	public static final double ELEVATOR_I_ZONE = 0.0; // Range in which I is used
	public static final double ELEVATOR_MIN_OUTPUT = -0.75;
	public static final double ELEVATOR_MAX_OUTPUT = 0.25; // Because the elevator is inverted, positive power goes
															// down. Due to gravity, we want to limit the power going
															// down, which this does
	public static final int HIGH_GEAR_PID_SLOT = 1;

	// Constants for low gear/climbing. ALWAYS ZERO TO PREVENT LOW GEAR POSITION
	// CONTROL
	public static final double ELEVATOR_CLIMB_P = 0.0;
	public static final double ELEVATOR_CLIMB_I = 0.0;
	public static final double ELEVATOR_CLIMB_D = 0.0;
	public static final double ELEVATOR_CLIMB_F = 0.0;
	public static final double ELEVATOR_CLIMB_I_ZONE = 0.0; // Range in which I is used
	public static final int LOW_GEAR_PID_SLOT = 0;

	/** Elevator Setpoints */
	// TODO these are all placeholder values. They should be tuned tomorrow/Monday
	public static final double HATCH_LEVEL_1_HEIGHT = 1.0;
	public static final double HATCH_LEVEL_2_HEIGHT = 26.0;
	public static final double HATCH_LEVEL_3_HEIGHT = 45.0;
	public static final double CARGO_SHIP_HEIGHT = 20.0;
	public static final double CARGO_LEVEL_1_HEIGHT = 1.0;
	public static final double CARGO_LEVEL_2_HEIGHT = 25.5;
	public static final double CARGO_LEVEL_3_HEIGHT = 45.95;

	/** CLIMB CONSTANTS */
	public static final double WINCH_RAISE_POWER = 0.6;
	public static final double WINCH_LOWER_POWER = -0.6;
	public static final boolean IS_RIGHT_WINCH_INVERTED = false;
	public static final boolean IS_LEFT_WINCH_INVERTED = true;

	public static final double VACUUM_POWER = 0.5;
	public static final boolean IS_VACUUM_MOTOR_INVERTED = false;
	public static final double PRESSURE_SENSOR_RESISTANCE = 250.0; // Ohms, Needs Testing
	public static final double MAX_PSI_READING = 14.5;
	public static final double MAX_MILLIAMP_READING = 20.0;
	public static final double MILLIAMPS_TO_PSI = MAX_PSI_READING / MAX_MILLIAMP_READING;

	/** INPUT CONSTANTS */
	public static final double ELEVATOR_MANUAL_POWER_SCALAR = 1.0;
	public static final double CARGO_INTAKE_INPUT_MAGNITUDE = 1.0;
	/**
	 * Seconds
	 */
	public static final double HATCH_EJECT_RETRACT_TIMEOUT = 0.3;
}