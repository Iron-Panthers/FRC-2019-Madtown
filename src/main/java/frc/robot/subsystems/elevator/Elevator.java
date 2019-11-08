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
	private final SparkMaxMotorGroup m_elevatorMotors;
	private final Solenoid m_elevatorShift;
	private final CANDigitalInput m_topLimit, m_bottomLimit;

	public Elevator() {
		final CANSparkMax elevatorMotor1 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M1, MotorType.kBrushless);
		final CANSparkMax elevatorMotor2 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M2, MotorType.kBrushless);
		final CANSparkMax elevatorMotor3 = new CANSparkMax(Constants.CANIDs.ELEVATOR_M3, MotorType.kBrushless);

		m_elevatorMotors = new SparkMaxMotorGroup("Elevator", elevatorMotor1, elevatorMotor2, elevatorMotor3);

		m_elevatorShift = new Solenoid(Constants.PCMIDs.ELEVATOR_SHIFT);

		// Use limit switch(es) with master motor controller at the "board level"
		// This prevents movement upwards/positive percentoutputs when the top limit is
		// triggered, and does the same for negative percentoutputs when the bottom is
		// triggered.
		m_topLimit = m_elevatorMotors.getMasterMotor().getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		m_bottomLimit = m_elevatorMotors.getMasterMotor().getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		m_topLimit.enableLimitSwitch(true);
		m_bottomLimit.enableLimitSwitch(true);

		m_elevatorMotors.setInverted(true);

		// Populate PID slots with the low-gear and high-gear gains
		configureHighGearPID();
		configureLowGearPID();
	}

	/**
	 * Configure the elevator using the PID for high gear/scoring.
	 */
	public void configureHighGearPID() {
		CANPIDController pidController = m_elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_P, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setI(Constants.ELEVATOR_I, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setD(Constants.ELEVATOR_D, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setIZone(Constants.ELEVATOR_I_ZONE, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setFF(Constants.ELEVATOR_F, Constants.HIGH_GEAR_PID_SLOT);
		pidController.setOutputRange(Constants.ELEVATOR_MIN_OUTPUT, Constants.ELEVATOR_MAX_OUTPUT,
				Constants.HIGH_GEAR_PID_SLOT);
	}

	/**
	 * Configure the elevator using the PID for low gear/climbing.
	 */
	public void configureLowGearPID() {
		CANPIDController pidController = m_elevatorMotors.getMasterMotor().getPIDController();
		pidController.setP(Constants.ELEVATOR_CLIMB_P, Constants.LOW_GEAR_PID_SLOT);
		pidController.setI(Constants.ELEVATOR_CLIMB_I, Constants.LOW_GEAR_PID_SLOT);
		pidController.setD(Constants.ELEVATOR_CLIMB_D, Constants.LOW_GEAR_PID_SLOT);
		pidController.setIZone(Constants.ELEVATOR_CLIMB_I_ZONE, Constants.LOW_GEAR_PID_SLOT);
		pidController.setFF(Constants.ELEVATOR_CLIMB_F, Constants.LOW_GEAR_PID_SLOT);
		pidController.setOutputRange(Constants.ELEVATOR_LOW_GEAR_MIN_OUTPUT, Constants.ELEVATOR_LOW_GEAR_MAX_OUTPUT, Constants.LOW_GEAR_PID_SLOT);
	}

	public void configureOhCrapPID() {
		configureHighGearPID();
		CANPIDController pidController = m_elevatorMotors.getMasterMotor().getPIDController();
		pidController.setOutputRange(-1.0, 1.0, Constants.HIGH_GEAR_PID_SLOT);
	}

	/**
	 * @return Whether or not the elevator is in high gear.
	 */
	public boolean getGearState() {
		return m_elevatorShift.get();
	}

	/**
	 * Set the target for the motor using the CANSparkMax position control. Uses the
	 * slot according to the gear state
	 * 
	 * @param rotations Encoder rotations/revolutions demand.
	 */
	public void setTarget(double rotations) {
		final int pidSlot = getGearState() ? Constants.HIGH_GEAR_PID_SLOT : Constants.LOW_GEAR_PID_SLOT;
		// Use the correct top limit position depending on high gear or low gear
		if (getGearState()) {
			if (m_topLimit.get()) {
				setPosition(Constants.TOP_LIMIT_POSITION);
			} else if (m_bottomLimit.get()) {
				setPosition(Constants.BOTTOM_LIMIT_POSITION);
			}
		} else if (!getGearState()) {
			if (m_topLimit.get()) {
				setPosition(Constants.TOP_LOW_GEAR_LIMIT_POSITION);
			} else if (m_bottomLimit.get()) {
				setPosition(Constants.BOTTOM_LIMIT_POSITION);
			}
		}

		// Set a controller reference value, using the desired number of rotations
		// We also specify ControlType.kPosition, which defines the correct underlying
		// control type to use.
		// PIDSlot defines the set of PID gains to be used to achieve/maintain the
		// position given.
		// The `0.0` at the end declares that there is no arbitrary feedforward.
		m_elevatorMotors.getMasterMotor().getPIDController().setReference(rotations, ControlType.kPosition, pidSlot,
				0.0);
	}

	/**
	 * Raise the elevator unless the top limit is pressed.
	 * 
	 * @param power Power between 0.0 and 1.0 for the power, absolute value used for
	 *              safety and for limit switches
	 */
	public void raise(double power) {
		// High Gear
		if (getGearState()) {
			if (Constants.TOP_LIMIT_POSITION
					- Robot.elevator.getPosition() < Constants.ELEVATOR_ROTATION_TOLERANCE_HIGH_GEAR) {
				m_elevatorMotors.set(Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				m_elevatorMotors.set(Math.abs(power));
			}
		}
		// Low Gear
		else if (!getGearState()) {
			if ((Constants.TOP_LOW_GEAR_LIMIT_POSITION
					- Robot.elevator.getPosition()) < Constants.ELEVATOR_ROTATION_TOLERANCE_LOW_GEAR) {
				m_elevatorMotors.set(Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				m_elevatorMotors.set(Math.abs(power));
			}
		}
	}

	/**
	 * Lower the elevator unless the bottom limit is pressed.
	 * 
	 * @param power Power between -1.0 and 0.0 for the power, absolute value used
	 *              for safety and for limit switches
	 */
	public void lower(double power) {
		// High Gear
		if (getGearState()) {
			if ((Math.abs(Constants.BOTTOM_LIMIT_POSITION
					- Robot.elevator.getPosition())) < Constants.ELEVATOR_ROTATION_TOLERANCE_HIGH_GEAR) {
				m_elevatorMotors.set(-Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				m_elevatorMotors.set(-Math.abs(power));
			}
		}
		// Low Gear
		else if (!getGearState()) {
			if ((Math.abs(Constants.BOTTOM_LIMIT_POSITION
					- Robot.elevator.getPosition())) < Constants.ELEVATOR_ROTATION_TOLERANCE_LOW_GEAR) {
				m_elevatorMotors.set(-Math.abs(power) * Constants.ROTATION_TOLERANCE_MULTIPLIER);
			} else {
				m_elevatorMotors.set(-Math.abs(power));
			}
		}
	}

	public void stop() {
		m_elevatorMotors.set(0.0);
		Robot.elevator.setTarget(Robot.elevator.getPosition());
	}

	public void shiftHigh() {
		if (getGearState()) {
			return;
		} else {
			m_elevatorShift.set(true);
			convertPositionToHighGear();
		}
	}

	public void shiftLow() {
		if (!getGearState()) {
			return;
		} else {
			m_elevatorShift.set(false);
			convertPositionToLowGear();
		}
	}

	public double getPosition() {
		return m_elevatorMotors.getEncoderPosition();
	}

	/**
	 * Set the position of the encoder, used for recalibration
	 * 
	 * @param rotations Rotations to use for the new encoder position
	 */
	public void setPosition(double rotations) {
		m_elevatorMotors.getMasterMotor().setEncPosition(rotations);
	}

	/**
	 * Convert encoder position from high gear position to the equivalent position
	 * in low gear, used during shifting
	 */
	private void convertPositionToLowGear() {
		m_elevatorMotors.getMasterMotor()
				.setEncPosition(m_elevatorMotors.getEncoderPosition() * Constants.HIGH_GEAR_TO_LOW_GEAR_ROTATIONS);
	}

	/**
	 * Convert encoder position from low gear position to the equivalent position in
	 * high gear, used during shifting
	 */
	private void convertPositionToHighGear() {
		m_elevatorMotors.getMasterMotor()
				.setEncPosition(m_elevatorMotors.getEncoderPosition() * Constants.LOW_GEAR_TO_HIGH_GEAR_ROATIONS);
	}

	@Override
	public void initDefaultCommand() {
		// No default needed because setting to target keeps going even when a command
		// is not running
	}
}
