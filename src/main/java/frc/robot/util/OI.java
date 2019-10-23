/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.subsystems.drive.commands.DriveShift;
import frc.robot.subsystems.drive.commands.ReverseDrive;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public JoystickWrapper stick1;
	public JoystickButton reverseDrive, driveShift;

	public OI() {
		stick1 = new JoystickWrapper(Constants.JOYSTICK_1_PORT);
		reverseDrive = new JoystickButton(stick1, Constants.REVERSE_DRIVE_PORT);
		driveShift = new JoystickButton(stick1, Constants.DRIVE_SHIFT_PORT);

		reverseDrive.whileHeld(new ReverseDrive());
		driveShift.whileHeld(new DriveShift());
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
