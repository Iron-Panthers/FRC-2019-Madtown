/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Climb extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private final Solenoid climbRelease;
	private final TalonSRX leftWinch;
	private final TalonSRX rightWinch;
	private final TalonSRX vacuumMotor;

	public Climb() {
		climbRelease = new Solenoid(Constants.PCMIDs.CLIMB_RELEASE);
		leftWinch = new TalonSRX(Constants.CANIDs.LEFT_WINCH);
		leftWinch.setInverted(Constants.IS_LEFT_WINCH_INVERTED);
		rightWinch = new TalonSRX(Constants.CANIDs.RIGHT_WINCH);
		rightWinch.setInverted(Constants.IS_RIGHT_WINCH_INVERTED);
		vacuumMotor = new TalonSRX(Constants.CANIDs.VACUUM_PORT);
	}

	public void raiseLeftWinch() {
		leftWinch.set(ControlMode.PercentOutput, Math.abs(Constants.WINCH_RAISE_POWER));
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
