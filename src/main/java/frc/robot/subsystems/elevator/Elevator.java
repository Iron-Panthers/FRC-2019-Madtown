/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private SparkMaxMotorGroup elevatorMotors;
	private Solenoid elevatorShift, extendArm;
	private CANDigitalInput topLimit, bottomLimit;
	private GearState elevatorGearState;

	// Gear State is used to know which pid mode to use
	public enum GearState {
		HIGH, LOW
	}

	public Elevator() {
		elevatorMotors = Robot.hardware.elevatorMotors;
		elevatorShift = Robot.hardware.elevatorShift;
		extendArm = Robot.hardware.extendArm;
		topLimit = elevatorMotors.getMasterMotor().getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		bottomLimit = elevatorMotors.getMasterMotor().getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		topLimit.enableLimitSwitch(true);
		bottomLimit.enableLimitSwitch(true);
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
		// Adjust gear state for PID
		if (elevatorGearState == GearState.HIGH) {
			pidSlot = Constants.HIGH_GEAR_PID_SLOT;
		} else if (elevatorGearState == GearState.LOW) {
			pidSlot = Constants.LOW_GEAR_PID_SLOT;
		} else {
			System.out.println("Elevator Gear State Error");
		}
		if (topLimit.get()) {
			setPosition(Constants.TOP_LIMIT_POSITION);
		} else if (bottomLimit.get()) {
			setPosition(Constants.BOTTOM_LIMIT_POSITION);
		}
		// This will stop in either direction if the limit switch is pressed
		elevatorMotors.getMasterMotor().getPIDController().setReference(rotations, ControlType.kPosition, pidSlot);
	}

	/**
	 * Raise the elevator unless the top limit is pressed
	 * 
	 * @param power Power between 0.0 and 1.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void raise(double power) {
		// This should be redundant due to the use of a limit switch attached directly
		// to the master motor
		if (!topLimit.get()) {
			// Scale down the power if close to the top. No Math.abs in case the position is
			// significantly above the top limit due to encoder slippage
			if ((Constants.TOP_LIMIT_POSITION - Robot.elevator.getPosition()) < Constants.ELEVATOR_ROTATION_TOLERANCE) {
				elevatorMotors.set(Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				elevatorMotors.set(Math.abs(power));
			}
		} else {
			// Stop motors and recalibrate the encoder
			stop();
			setPosition(Constants.TOP_LIMIT_POSITION);
		}
	}

	/**
	 * Lower the elevator unless the bottom limit is pressed
	 * 
	 * @param power Power between -1.0 and 0.0 for the power, absolute value used
	 *              for safety and for limit switches
	 */
	public void lower(double power) {
		// This should be redundant due to the use of a limit switch attached directly
		// to the master motor
		if (!bottomLimit.get()) {
			// Scale down the power if close to the bottom
			if ((Math.abs(Constants.BOTTOM_LIMIT_POSITION
					- Robot.elevator.getPosition())) < Constants.ELEVATOR_ROTATION_TOLERANCE) {
				elevatorMotors.set(-Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				elevatorMotors.set(-Math.abs(power));
			}
		} else {
			stop();
			setPosition(Constants.BOTTOM_LIMIT_POSITION);
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

	/**
	 * Set the position of the encoder, used for recalibration
	 * 
	 * @param rotations Rotations to use for the new encoder position
	 */
	public void setPosition(double rotations) {
		elevatorMotors.getMasterMotor().setEncPosition(rotations);
	}

	@Override
	public void initDefaultCommand() {
		// No default needed because setting to target keeps going even when a command
		// is not running
	}
}
