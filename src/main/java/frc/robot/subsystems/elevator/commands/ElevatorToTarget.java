/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ElevatorToTarget extends InstantCommand {
	private double target;
	/**
	 * Changes target of subsystem to desired target
	 */
	public ElevatorToTarget(double target) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		this.target = target;
	}

	// Called once when the command executes
	@Override
	protected void initialize() {
		Robot.elevator.setTarget(target);
	}

}
