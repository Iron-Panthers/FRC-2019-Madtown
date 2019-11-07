package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class OuttakeHatch extends TimedCommand {
    public OuttakeHatch(double timeout) {
        super(timeout);
        requires(Robot.superstructure);
    }

    @Override
    protected void initialize() {
        // First, release the hatch fingers, then use the hatch ejectors.
        // releasing = true
        Robot.superstructure.useHatchFingers(true);
        // extending = true
        Robot.superstructure.useHatchEjectors(true);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        // Retract the outtake pistons after the timeout
        Robot.superstructure.useHatchEjectors(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
