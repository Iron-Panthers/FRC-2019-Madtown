package frc.robot.subsystems.superstructure.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.elevator.commands.ElevatorToTarget;

public class ElevatorAndIntakeHeight extends CommandGroup {
    /**
     * A "compounded" Command, where an elevator movement is followed by a
     * raising/lowering of the intake.
     * 
     * @param elevatorTarget The setpoint (rotations/position) for the elevator.
     * @param intakeHigh Whether or not the intake "arm" should be in its high state or not.
     */
    public ElevatorAndIntakeHeight(double elevatorTarget, boolean intakeHigh) {
        System.out.println("ElevatorHeight: " + elevatorTarget + ", IntakeHigh: " + intakeHigh);

        addSequential(new ElevatorToTarget(elevatorTarget));
        addSequential(new IntakeHeightCommand(intakeHigh));
    }
}