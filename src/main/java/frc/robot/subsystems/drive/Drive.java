/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.commands.ArcadeDrive;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * The drive subsystem. This contains MotorGroups for the left and right
 * drivebase motors.
 */
public class Drive extends Subsystem {
	private final SparkMaxMotorGroup left;
	private final SparkMaxMotorGroup right;
	private final Solenoid gearShift;
	private boolean isReversed;

	/**
	 * Create the drivebase subsystem. This sets the inversion status of the left
	 * and right drive motorgroups to values specified in constants.
	 */
	public Drive() {
		final CANSparkMax driveLeft1 = new CANSparkMax(Constants.CANIDs.DRIVE_L1_PORT, MotorType.kBrushless);
		final CANSparkMax driveLeft2 = new CANSparkMax(Constants.CANIDs.DRIVE_L2_PORT, MotorType.kBrushless);
		final CANSparkMax driveRight1 = new CANSparkMax(Constants.CANIDs.DRIVE_R1_PORT, MotorType.kBrushless);
		final CANSparkMax driveRight2 = new CANSparkMax(Constants.CANIDs.DRIVE_R2_PORT, MotorType.kBrushless);

		left = new SparkMaxMotorGroup("Elevator/left", driveLeft1, driveLeft2);
		right = new SparkMaxMotorGroup("Elevator/right", driveRight1, driveRight2);

		gearShift = new Solenoid(Constants.PCMIDs.DRIVE_GEAR_SHIFT);

		left.setInverted(Constants.IS_LEFT_INVERTED);
		right.setInverted(Constants.IS_RIGHT_INVERTED);
		isReversed = false;
	}

	/**
	 * My powers have doubled since we last met. Set the power of MotorGroups in the
	 * drivebase.
	 * 
	 * @param leftPower  the power to set for the left motor group.
	 * @param rightPower the power to set for the right motor group.
	 */
	public void set(double leftPower, double rightPower) {
		left.set(leftPower);
		right.set(rightPower);
		SmartDashboard.putNumber("Drive -- Set left power: ", leftPower);
		SmartDashboard.putNumber("Drive -- Set right power: ", rightPower);
	}

	/**
	 * Sets the power of both sides of the robot to the same side
	 * 
	 * @param power The power to set for the left and right motor group
	 */
	public void set(double power) {
		left.set(power);
		right.set(power);
		SmartDashboard.putNumber("Drive -- Set left power: ", power);
		SmartDashboard.putNumber("Drive -- Set right power: ", power);
	}

	/**
	 * Things the subsystem should do at init of new phases.
	 */
	public void reset() {
		left.stop();
		right.stop();
	}

	/**
	 * @return The velocity of the motor in RPM
	 */
	public double getLeftVelocity() {
		return left.getMasterMotor().getEncoder().getVelocity();
	}

	/**
	 * @return The velocity of the motor in RPM
	 */
	public double getRightVelocity() {
		return right.getMasterMotor().getEncoder().getVelocity();
	}

	/**
	 * @return the encoder position of the left encoder, in encoder revolutions.
	 */
	public double getLeftEncoderRevolutions() {
		return left.getEncoderPosition();
	}

	/**
	 * @return the encoder position of the right encoder, in encoder revolutions.
	 */
	public double getRightEncoderRevolutions() {
		return right.getEncoderPosition();
	}

	public boolean getReversed() {
		return isReversed;
	}

	public void setReversed(boolean isReversed) {
		this.isReversed = isReversed;
	}

	/**
	 * Shift the drivebase to low gear.
	 */
	public void shiftLow() {
		gearShift.set(true);
	}

	/**
	 * Shift the drivebase to high gear.
	 */
	public void shiftHigh() {
		gearShift.set(false);
	}

	@Override
	public void initDefaultCommand() {
		// Pick one of the drive mode commands.
		setDefaultCommand(new ArcadeDrive());
	}
}