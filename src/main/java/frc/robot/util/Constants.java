package frc.robot.util;

public class Constants {

	// File for Constants, ports and values
	/** DRIVEBASE PORTS */
	public static final int DRIVE_R1_PORT = 1; // SparkMax
	public static final int DRIVE_R2_PORT = 21; // See Above
	public static final int DRIVE_L1_PORT = 2; // See Above
	public static final int DRIVE_L2_PORT = 22; // See Above
	public static final int GEAR_SHIFT_PORT = 6;

	public static final boolean IS_LEFT_INVERTED = true;
	public static final boolean IS_RIGHT_INVERTED = false;
	public static final boolean IS_DRIVEBASE_BACKWARDS = true; // Needed so the robot actually thinks the front is
																// the front
	public static final double TURN_SENSITIVITY = 1;
	public static final double RADIAL_TURN_SENSITIVITY = 20;
	public static final double SCALING_POWER = 2.75;
	public static final double RAMP_RATE = 0.25; // Seconds to go from 0 to full throttle

	/** ELEVATOR PORTS */
	public static final int ELEVATOR_M1_PORT = 12;
	public static final int ELEVATOR_M2_PORT = 11;
	public static final int ELEVATOR_M3_PORT = 10;
	public static final int ELEVATOR_SHIFT_PORT = 4; // TBD, Placeholder Value
	public static final int EXTEND_ELEVATOR_PORT = 8; // TBD, Placeholder Value
	public static final int TOP_LIMIT_SWITCH_PORT = 0; // TBD, Placeholder
	public static final int BOTTOM_LIMIT_SWITCH_PORT = 1; // TBD, Placeholder

	/** Elevator Constants */
	public static final double BOTTOM_LIMIT_POSITION = 0.0; // In rotations by default but using conversion factors we
															// can make this into inches/meters
	public static final double TOP_LIMIT_POSITION = 1000.0; // TBD
	public static final double ELEVATOR_ROTATION_TOLERANCE = 20.0; // Rotations away from the ends of the elevator to slow down
	public static final double ROTATION_TOLERANCE_MULTIPLIER = 0.125; // TBD, Placeholder
	// Constants for high gear/scoring
	public static final double ELEVATOR_P = 0.0; // TODO: Tune. The plan is to use the sample code with shuffleboard
	public static final double ELEVATOR_I = 0.0;
	public static final double ELEVATOR_D = 0.0;
	public static final double ELEVATOR_F = 0.0;
	public static final double ELEVATOR_I_ZONE = 0.0; // Range in which I is used
	public static final int HIGH_GEAR_PID_SLOT = 1;
	// Constants for low gear/climbing
	// All of these are 0 to prevent use of setpoints in low gear
	public static final double ELEVATOR_CLIMB_P = 0.0; // TODO: Tune. The plan is to use the sample code with
														// shuffleboard
	public static final double ELEVATOR_CLIMB_I = 0.0;
	public static final double ELEVATOR_CLIMB_D = 0.0;
	public static final double ELEVATOR_CLIMB_F = 0.0;
	public static final double ELEVATOR_CLIMB_I_ZONE = 0.0; // Range in which I is used
	public static final int LOW_GEAR_PID_SLOT = 0;

	/** Elevator Setpoints */
	public static final double HATCH_LEVEL_1_HEIGHT = 100.0; // TBD, Placeholder Value
	public static final double HATCH_LEVEL_2_HEIGHT = 300.0; // TBD, Placeholder
	public static final double HATCH_LEVEL_3_HEIGHT = 500.0; // TBD
	public static final double CARGO_SHIP_HEIGHT = 200.0; // TBD, Placeholder
	public static final double CARGO_LEVEL_1_HEIGHT = 150.0; // TBD
	public static final double CARGO_LEVEL_2_HEIGHT = 350.0; // TBD
	public static final double CARGO_LEVEL_3_HEIGHT = 600.0; // TBD

	/** JOYSTICK PORTS */
	/** Driver A */
	public static final int JOYSTICK_1_PORT = 0; // Driver A
	public static final int REVERSE_DRIVE_PORT = 1;
	public static final int DRIVE_SHIFT_PORT = 2;

	/** Driver B */
	public static final int JOYSTICK_2_PORT = 1; // Driver B
	public static final int MANUAL_ELEVATOR_BUTTON = 1;
	public static final int INTAKE_BUTTON = 2;
	public static final int OUTTAKE_BUTTON = 3;
	public static final int HATCH_INTAKE_BUTTON = 5;
	public static final int HATCH_OUTTAKE_BUTTON = 6;

	/** Cargo Ship Setpoints */
	public static final int CARGO_SHIP_BUTTON = 4;

	/** Rocket Setpoints */
	public static final int HATCH_LEVEL_3_BUTTON = 7;
	public static final int HATCH_LEVEL_2_BUTTON = 9;
	public static final int HATCH_LEVEL_1_BUTTON = 11;
	public static final int CARGO_LEVEL_3_BUTTON = 8;
	public static final int CARGO_LEVEL_2_BUTTON = 10;
	public static final int CARGO_LEVEL_1_BUTTON = 12;

	/** INPUT CONSTANTS */
	public static final double ELEVATOR_MANUAL_POWER_SCALAR = 0.35;
	// Thresholds for shifting the elevator using the Z axis on the Joysticks, which
	// is the throttle. This is to save another button on the very cramped elevator
	// joystick
	// TODO: Test if throttle reads from -1 to 1 or from 0 to 1
	public static final double HIGH_GEAR_SHIFT_THRESHOLD = 0.7;
	public static final double LOW_GEAR_SHIFT_THRESHOLD = -0.7;
}