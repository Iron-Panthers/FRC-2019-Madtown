/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class OhCrap extends Command {
	public OhCrap() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.shiftHigh();
		Robot.elevator.configureOhCrapPID();
		Robot.superstructure.setIntakeHeight(false);
		Robot.elevator.setTarget(Constants.BOTTOM_LIMIT_POSITION);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return (Robot.elevator.getPosition() - Constants.BOTTOM_LIMIT_POSITION) < Constants.OH_CRAP_TOLERANCE; 
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.elevator.configureHighGearPID();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
