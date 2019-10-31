package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
	public CANSparkMax driveRight1, driveRight2;
	public CANSparkMax driveLeft1, driveLeft2;

	/* Drivebase MotorGroups */
	public SparkMaxMotorGroup rightDriveMotors, leftDriveMotors;

	public Solenoid gearShift;
	/** Motors/sensors for other subsystems will go down here */

	/* Elevator Motor Controllers */
	public CANSparkMax elevatorMotor1, elevatorMotor2, elevatorMotor3;
	public Solenoid elevatorShift, extendArm;

	/* Elevator MotorGroup */
	public SparkMaxMotorGroup elevatorMotors;

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

		/* Elevator motor controller creation */
		elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_M1_PORT, MotorType.kBrushless);
		elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_M2_PORT, MotorType.kBrushless);
		elevatorMotor3 = new CANSparkMax(Constants.ELEVATOR_M3_PORT, MotorType.kBrushless);

		elevatorMotors = new SparkMaxMotorGroup("Elevator Motor Group", elevatorMotor1, elevatorMotor2, elevatorMotor3);
		elevatorShift = new Solenoid(Constants.ELEVATOR_SHIFT_PORT);
		extendArm = new Solenoid(Constants.EXTEND_ELEVATOR_PORT);
	}
}
