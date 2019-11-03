package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class IntakeHeightCommand extends InstantCommand {
    private final boolean isHigh;

    public IntakeHeightCommand(boolean isHigh) {
        super();
        requires(Robot.superstructure);

        this.isHigh = isHigh;
    }

    @Override
    protected void initialize() {
        Robot.superstructure.setIntakeHeight(isHigh);
    }
}
