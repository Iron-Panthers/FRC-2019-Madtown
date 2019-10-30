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
	private GearState elevatorGearState;

	// Gear State is used to know which pid mode to use
	public enum GearState {
		HIGH, LOW
	}

	public Elevator() {
		elevatorMotors = Robot.hardware.elevatorMotors;
		elevatorShift = Robot.hardware.elevatorShift;
		extendArm = Robot.hardware.extendArm;
		topLimit = Robot.hardware.topLimit;
		bottomLimit = Robot.hardware.bottomLimit;
		elevatorGearState = GearState.HIGH; // To be tested
		setupScoring();
		setupClimbing();
	}

	/**
	 * Configure the elevator using the PID for high gear/scoring
	 */
	public void setupScoring() {
		CANPIDController pidController = elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_P, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setI(Constants.ELEVATOR_I, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setD(Constants.ELEVATOR_D, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setIZone(Constants.ELEVATOR_I_ZONE, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setFF(Constants.ELEVATOR_F, Constants.HIGH_GEAR_PID_SLOT);
	}

	/**
	 * Configure the elevator using the PID for low gear/climbing
	 */
	public void setupClimbing() {
		CANPIDController pidController = elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_CLIMB_P, Constants.LOW_GEAR_PID_SLOT);
		pidController.setI(Constants.ELEVATOR_CLIMB_I, Constants.LOW_GEAR_PID_SLOT);
		pidController.setD(Constants.ELEVATOR_CLIMB_D, Constants.LOW_GEAR_PID_SLOT);
		pidController.setIZone(Constants.ELEVATOR_CLIMB_I_ZONE, Constants.LOW_GEAR_PID_SLOT);
		pidController.setFF(Constants.ELEVATOR_CLIMB_F, Constants.LOW_GEAR_PID_SLOT);
	}

	/**
	 * Set the target for the motor using the CANSparkMax position control. Uses the
	 * slot according to the gear state
	 * 
	 * @param rotations Rotations by default, but possible to change with the
	 *                  conversion factor
	 */
	public void setTarget(double rotations) {
		// Use the slot according to what gear the robot is in
		int pidSlot = 0;
		if (elevatorGearState == GearState.HIGH) {
			pidSlot = Constants.HIGH_GEAR_PID_SLOT;
		} else if (elevatorGearState == GearState.LOW) {
			pidSlot = Constants.LOW_GEAR_PID_SLOT;
		} else {
			System.out.println("Elevator Gear State Error");
		}
		elevatorMotors.getMasterMotor().getPIDController().setReference(rotations, ControlType.kPosition, pidSlot);
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
	 * @param power Power between -1.0 and 0.0 for the power, absolute value used
	 *              for safety and for limit switches
	 */
	public void lower(double power) {
		if (!bottomLimit.get()) {
			elevatorMotors.set(-Math.abs(power));
		}
	}

	public void stop() {
		elevatorMotors.set(0);
		Robot.elevator.setTarget(Robot.elevator.getPosition());
	}

	// Needs to be tested
	public void shiftHigh() {
		elevatorShift.set(true);
		elevatorGearState = GearState.HIGH;
	}

	public void shiftLow() {
		elevatorShift.set(false);
		elevatorGearState = GearState.LOW;
	}

	// Needs to be tested
	public void extendArm() {
		extendArm.set(true);
	}

	public void retractArm() {
		extendArm.set(false);
	}

	public double getPosition() {
		return elevatorMotors.getEncoderPosition();
	}

	@Override
	public void initDefaultCommand() {
		// No default needed because setting to target keeps going even when a command
		// is not running
	}
}
