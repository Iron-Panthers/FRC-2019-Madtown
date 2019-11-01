/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.drive.commands.DriveShift;
import frc.robot.subsystems.drive.commands.ReverseDrive;
import frc.robot.subsystems.elevator.commands.ElevatorToTarget;
import frc.robot.subsystems.elevator.commands.ManualElevator;
import frc.robot.util.JoystickWrapper;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public JoystickWrapper driveStick;
	public JoystickWrapper elevatorStick;
	public JoystickButton reverseDrive, driveShift;
	public JoystickButton manualElevator;
	public JoystickButton hatchLevel1, hatchLevel2, hatchLevel3, cargoLevel1, cargoLevel2, cargoLevel3, cargoShipHeight;
	public JoystickButton intakeCargo, outtakeCargo, intakeHatch, outtakeHatch;

	public OI() {
		/** DRIVER A */
		driveStick = new JoystickWrapper(Constants.JOYSTICK_1_PORT);
		reverseDrive = new JoystickButton(driveStick, Constants.REVERSE_DRIVE_PORT);
		driveShift = new JoystickButton(driveStick, Constants.DRIVE_SHIFT_PORT);

		reverseDrive.whileHeld(new ReverseDrive());
		driveShift.whileHeld(new DriveShift());

		/** DRIVER B */
		elevatorStick = new JoystickWrapper(Constants.JOYSTICK_2_PORT);
		manualElevator = new JoystickButton(elevatorStick, Constants.MANUAL_ELEVATOR_BUTTON);
		hatchLevel1 = new JoystickButton(elevatorStick, Constants.HATCH_LEVEL_1_BUTTON);
		hatchLevel2 = new JoystickButton(elevatorStick, Constants.HATCH_LEVEL_2_BUTTON);
		hatchLevel3 = new JoystickButton(elevatorStick, Constants.HATCH_LEVEL_3_BUTTON);
		cargoLevel1 = new JoystickButton(elevatorStick, Constants.CARGO_LEVEL_1_BUTTON);
		cargoLevel2 = new JoystickButton(elevatorStick, Constants.CARGO_LEVEL_2_BUTTON);
		cargoLevel3 = new JoystickButton(elevatorStick, Constants.CARGO_LEVEL_3_BUTTON);
		cargoShipHeight = new JoystickButton(elevatorStick, Constants.CARGO_SHIP_BUTTON);
		intakeCargo = new JoystickButton(elevatorStick, Constants.INTAKE_BUTTON);
		outtakeCargo = new JoystickButton(elevatorStick, Constants.OUTTAKE_BUTTON);
		intakeHatch = new JoystickButton(elevatorStick, Constants.HATCH_INTAKE_BUTTON);
		outtakeHatch = new JoystickButton(elevatorStick, Constants.HATCH_OUTTAKE_BUTTON);

		manualElevator.whileHeld(new ManualElevator());
		hatchLevel1.whenPressed(new ElevatorToTarget(Constants.HATCH_LEVEL_1_BUTTON));
		hatchLevel2.whenPressed(new ElevatorToTarget(Constants.HATCH_LEVEL_2_HEIGHT));
		hatchLevel3.whenPressed(new ElevatorToTarget(Constants.HATCH_LEVEL_3_HEIGHT));
		cargoLevel1.whenPressed(new ElevatorToTarget(Constants.CARGO_LEVEL_1_HEIGHT));
		cargoLevel2.whenPressed(new ElevatorToTarget(Constants.CARGO_LEVEL_2_HEIGHT));
		cargoLevel3.whenPressed(new ElevatorToTarget(Constants.CARGO_LEVEL_3_HEIGHT));
		cargoShipHeight.whenPressed(new ElevatorToTarget(Constants.CARGO_SHIP_HEIGHT));
		// intakeCargo.whileHeld(new )
	}
	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
