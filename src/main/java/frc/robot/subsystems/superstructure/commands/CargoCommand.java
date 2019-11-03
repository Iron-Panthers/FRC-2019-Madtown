package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CargoCommand extends Command {
    private final double intakeMotorSpeed;

    /**
     * Creates a new CargoCommand for intaking or outtaking cargo.
     * 
     * @param intaking  Whether or not the Command should intake cargo.
     * @param magnitude [-1.0 .. 1.0] The "power" of the intaking to use.
     */
    public CargoCommand(boolean intaking, double magnitude) {
        requires(Robot.superstructure);

        if (intaking) {
            intakeMotorSpeed = Math.abs(magnitude);
        } else {
            intakeMotorSpeed = -Math.abs(magnitude);
        }
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.superstructure.setCargoIntakePercentOutput(intakeMotorSpeed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.superstructure.setCargoIntakePercentOutput(0.0);
    }

    @Override
    protected void interrupted() {
        Robot.superstructure.setCargoIntakePercentOutput(0.0);
    }
}
