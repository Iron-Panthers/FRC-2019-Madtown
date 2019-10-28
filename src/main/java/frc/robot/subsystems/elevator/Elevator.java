/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private SparkMaxMotorGroup elevatorMotors;
	private Solenoid elevatorShift;
	private Solenoid extendArm;

	public Elevator() {
		elevatorMotors = Robot.hardware.elevatorMotors;
		elevatorShift = Robot.hardware.elevatorShift;
		extendArm = Robot.hardware.extendArm;
	}

	/**
	 * Raise the elevator
	 * 
	 * @param power Power between 0.0 and 1.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void raise(double power) {
		elevatorMotors.set(Math.abs(power));
	}

	/**
	 * Lower the elevator
	 * 
	 * @param power Power between -1.0 and 0.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void lower(double power) {
		elevatorMotors.set(-Math.abs(power));
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
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
