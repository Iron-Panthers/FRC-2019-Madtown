/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class ManualElevator extends Command {
	public ManualElevator() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Always use low gear for manual control for safety
		Robot.elevator.shiftLow(); 
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double y = Robot.oi.elevatorStick.getY();
		y *= Constants.ELEVATOR_MANUAL_POWER_SCALAR;
		// If positive, raise the elevator
		if (y > 0) {
			Robot.elevator.raise(y);
		}
		else {
			Robot.elevator.lower(y);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.elevator.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.elevator.stop();
	}
}
