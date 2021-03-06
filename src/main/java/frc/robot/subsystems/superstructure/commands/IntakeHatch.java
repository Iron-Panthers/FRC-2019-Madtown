/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class IntakeHatch extends InstantCommand {
    public IntakeHatch() {
        super();
        requires(Robot.superstructure);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.superstructure.useHatchFingers(false);
    }
}
