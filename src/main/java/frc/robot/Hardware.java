package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.util.Constants;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * This class is meant to store raw hardware instances. Examples: Motor
 * controllers, sensors, etc. Meant to contain hardware declarations that would
 * otherwise be in Robot class.
 */
public class Hardware {
	/* Drivebase motor controllers */
	public CANSparkMax driveRight1;
	public CANSparkMax driveRight2;
	public CANSparkMax driveLeft1;
	public CANSparkMax driveLeft2;

	/* Drivebase MotorGroups */
	public SparkMaxMotorGroup rightDriveMotors;
	public SparkMaxMotorGroup leftDriveMotors;

	public Solenoid gearShift;

	/** Motors/sensors for other subsystems will go down here */

	public Hardware() {
		/* Drivebase motor controller creation */
		driveRight1 = new CANSparkMax(Constants.DRIVE_R1_PORT, MotorType.kBrushless);
		driveRight2 = new CANSparkMax(Constants.DRIVE_R2_PORT, MotorType.kBrushless);
		driveLeft1 = new CANSparkMax(Constants.DRIVE_L1_PORT, MotorType.kBrushless);
		driveLeft2 = new CANSparkMax(Constants.DRIVE_L2_PORT, MotorType.kBrushless);

		/* Drivebase configuration */
		rightDriveMotors = new SparkMaxMotorGroup("Right Drive Motor Group", driveRight1, driveRight2);
		leftDriveMotors = new SparkMaxMotorGroup("Left Drive Motor Group", driveLeft1, driveLeft2);
		gearShift = new Solenoid(Constants.GEAR_SHIFT_PORT);
	}
}
