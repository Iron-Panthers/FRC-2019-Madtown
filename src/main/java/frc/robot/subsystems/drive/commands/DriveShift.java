/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive.commands;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveShift extends Command {
	
	public DriveShift() {
		// This command must not require a subsystem, otherwise it prevents drive from moving while shifting
	}
	
	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drive.shiftLow();
	}
	
	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}
	
	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}
	
	// Called once after isFinished returns true
	protected void end() {
		Robot.drive.shiftHigh();
	}
	
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.drive.shiftHigh();
	}
}
