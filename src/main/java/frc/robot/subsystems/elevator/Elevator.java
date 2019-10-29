/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private SparkMaxMotorGroup elevatorMotors;
	private Solenoid elevatorShift, extendArm;
	private DigitalInput topLimit, bottomLimit;

	public Elevator() {
		elevatorMotors = Robot.hardware.elevatorMotors;
		elevatorShift = Robot.hardware.elevatorShift;
		extendArm = Robot.hardware.extendArm;
		topLimit = Robot.hardware.topLimit;
		bottomLimit = Robot.hardware.bottomLimit;
		setup();
	}

	public void setup() {
		CANPIDController pidController = elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_P);
		pidController.setI(Constants.ELEVATOR_I);
		pidController.setD(Constants.ELEVATOR_D);
		pidController.setIZone(Constants.ELEVATOR_I_ZONE);
		pidController.setFF(Constants.ELEVATOR_F);
	}

	/**
	 * Set the target for the motor using the CANSparkMax position control
	 * @param rotations Rotations by default, but possible to change with the conversion factor
	 */
	public void setTarget(double rotations) {
		elevatorMotors.getMasterMotor().getPIDController().setReference(rotations, ControlType.kPosition);
	}

	/**
	 * Raise the elevator unless the top limit is pressed
	 * 
	 * @param power Power between 0.0 and 1.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void raise(double power) {
		if (!topLimit.get()) {
			elevatorMotors.set(Math.abs(power));
		}
	}

	/**
	 * Lower the elevator unless the bottom limit is pressed
	 * 
	 * @param power Power between -1.0 and 0.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void lower(double power) {
		if (!bottomLimit.get()) {
			elevatorMotors.set(-Math.abs(power));
		}
	}

	// Needs to be tested
	public void shiftHigh() {
		elevatorShift.set(true);
	}

	public void shiftLow() {
		elevatorShift.set(false);
	}

	// Needs to be tested
	public void extendArm() {
		extendArm.set(true);
	}

	public void retractArm() {
		extendArm.set(false);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new ElevatorHoldPosition());
	}
}
