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


	/** JOYSTICK PORTS */
	public static final int JOYSTICK_1_PORT = 0; // Driver A
	public static final int REVERSE_DRIVE_PORT = 1;
	public static final int DRIVE_SHIFT_PORT = 2;

}