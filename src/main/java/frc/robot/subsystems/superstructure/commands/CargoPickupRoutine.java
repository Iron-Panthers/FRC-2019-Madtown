/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;

public class CargoPickupRoutine extends CommandGroup {
    public CargoPickupRoutine() {
        addSequential(new ElevatorAndIntakeHeight(0.0, false));
        addSequential(new CargoCommand(true, Constants.CARGO_INTAKE_INPUT_MAGNITUDE));
    }
}
