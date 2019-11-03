/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SparkMaxMotorGroup;

/**
 * The subsystem responsible for controlling the robot's elevator.
 */
public class Elevator extends Subsystem {
	private final SparkMaxMotorGroup elevatorMotors;
	private final Solenoid elevatorShift;
	private final CANDigitalInput topLimit, bottomLimit;
	private GearState elevatorGearState;

	// Gear State is used to know which pid mode to use
	public enum GearState {
		HIGH, LOW
	}

	public Elevator() {
		final CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M1, MotorType.kBrushless);
		final CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M2, MotorType.kBrushless);
		final CANSparkMax elevatorMotor3 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M3, MotorType.kBrushless);

		elevatorMotors = new SparkMaxMotorGroup("Elevator Motor Group", elevatorMotor1, elevatorMotor2, elevatorMotor3);

		elevatorShift = new Solenoid(Constants.PCMIDs.ELEVATOR_SHIFT);

		// Use limit switch(es) with master motor controller at the "board level"
		// This prevents movement upwards/positive percentoutputs when the top limit is
		// triggered, and does the same for negative percentoutputs when the bottom is
		// triggered.
		topLimit = elevatorMotors.getMasterMotor().getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		bottomLimit = elevatorMotors.getMasterMotor().getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		topLimit.enableLimitSwitch(true);
		bottomLimit.enableLimitSwitch(true);

		elevatorMotors.setInverted(false);

		// Assume that the elevator is in its high gear when the robot program
		// initializes
		elevatorGearState = GearState.HIGH;

		// Populate PID slots with the low-gear and high-gear gains
		configureHighGearPID();
		configureLowGearPID();
	}

	/**
	 * Configure the elevator using the PID for high gear/scoring.
	 */
	public void configureHighGearPID() {
		CANPIDController pidController = elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_P, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setI(Constants.ELEVATOR_I, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setD(Constants.ELEVATOR_D, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setIZone(Constants.ELEVATOR_I_ZONE, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setFF(Constants.ELEVATOR_F, Constants.HIGH_GEAR_PID_SLOT);
	}

	/**
	 * Configure the elevator using the PID for low gear/climbing.
	 */
	public void configureLowGearPID() {
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
	 * @param rotations Encoder rotations/revolutions demand.
	 */
	public void setTarget(double rotations) {
		final int pidSlot = elevatorGearState == GearState.HIGH ? Constants.HIGH_GEAR_PID_SLOT
				: Constants.LOW_GEAR_PID_SLOT;

		if (topLimit.get()) {
			setPosition(Constants.TOP_LIMIT_POSITION);
		} else if (bottomLimit.get()) {
			setPosition(Constants.BOTTOM_LIMIT_POSITION);
		}

		// Set a controller reference value, using the desired number of rotations
		// We also specify ControlType.kPosition, which defines the correct underlying
		// control type to use.
		// PIDSlot defines the set of PID gains to be used to achieve/maintain the
		// position given.
		// The `0.0` at the end declares that there is no arbitrary feedforward.
		elevatorMotors.getMasterMotor().getPIDController().setReference(rotations, ControlType.kPosition, pidSlot, 0.0);
	}

	/**
	 * Raise the elevator unless the top limit is pressed
	 * 
	 * @param power Power between 0.0 and 1.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void raise(double power) {
		if ((Constants.TOP_LIMIT_POSITION - Robot.elevator.getPosition()) < Constants.ELEVATOR_ROTATION_TOLERANCE) {
			elevatorMotors.set(Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
		} else {
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
		if ((Math.abs(Constants.BOTTOM_LIMIT_POSITION
				- Robot.elevator.getPosition())) < Constants.ELEVATOR_ROTATION_TOLERANCE) {
			elevatorMotors.set(-Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
		} else {
			elevatorMotors.set(-Math.abs(power));
		}
	}

	public void stop() {
		elevatorMotors.set(0.0);
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
