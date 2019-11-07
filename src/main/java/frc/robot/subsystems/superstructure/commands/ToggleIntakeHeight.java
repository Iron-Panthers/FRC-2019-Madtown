package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class ToggleIntakeHeight extends InstantCommand {
    public ToggleIntakeHeight() {
        super();
        requires(Robot.superstructure);
    }

    @Override
    protected void initialize() {
        Robot.superstructure.toggleIntakeHeight();
    }
}
