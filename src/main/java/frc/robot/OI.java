package frc.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.drive.commands.DriveShift;
import frc.robot.subsystems.drive.commands.ReverseDrive;
import frc.robot.subsystems.elevator.commands.ManualElevator;
import frc.robot.subsystems.superstructure.commands.CargoCommand;
import frc.robot.subsystems.superstructure.commands.ElevatorAndIntakeHeight;
import frc.robot.subsystems.superstructure.commands.IntakeHatch;
import frc.robot.subsystems.superstructure.commands.OuttakeHatch;
import frc.robot.util.JoystickWrapper;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public final JoystickWrapper driverAController;
	public final JoystickWrapper driverBController;
	public final JoystickButton reverseDrive, driveShift;
	public final JoystickButton useManualElevator;
	public final JoystickButton hatchLevel1;
	public final JoystickButton hatchLevel2;
	public final JoystickButton hatchLevel3;
	public final JoystickButton cargoLevel1;
	public final JoystickButton cargoLevel2;
	public final JoystickButton cargoLevel3;
	public final JoystickButton cargoShipHeight;
	public final JoystickButton intakeCargo;
	public final JoystickButton outtakeCargo;
	public final JoystickButton intakeHatch;
	public final JoystickButton outtakeHatch;

	public OI() {
		/** DRIVER A */
		driverAController = new JoystickWrapper(0);
		reverseDrive = new JoystickButton(driverAController, 1);
		driveShift = new JoystickButton(driverAController, 2);

		/** DRIVER B */
		driverBController = new JoystickWrapper(1);
		useManualElevator = new JoystickButton(driverBController, 1);
		hatchLevel1 = new JoystickButton(driverBController, 11);
		hatchLevel2 = new JoystickButton(driverBController, 9);
		hatchLevel3 = new JoystickButton(driverBController, 7);
		cargoLevel1 = new JoystickButton(driverBController, 12);
		cargoLevel2 = new JoystickButton(driverBController, 10);
		cargoLevel3 = new JoystickButton(driverBController, 8);
		cargoShipHeight = new JoystickButton(driverBController, 4);

		intakeCargo = new JoystickButton(driverBController, 2);
		outtakeCargo = new JoystickButton(driverBController, 3);
		intakeHatch = new JoystickButton(driverBController, 5);
		outtakeHatch = new JoystickButton(driverBController, 6);

		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
	 * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
	 * {@link edu.wpi.first.wpilibj.XboxController}), and passing these GenericHID
	 * to {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		/** DRIVER A */
		reverseDrive.whileHeld(new ReverseDrive());
		driveShift.whileHeld(new DriveShift());

		/** DRIVER B */
		useManualElevator.whileHeld(new ManualElevator());
		hatchLevel1.whenPressed(new ElevatorAndIntakeHeight(Constants.HATCH_LEVEL_1_HEIGHT, true));
		hatchLevel2.whenPressed(new ElevatorAndIntakeHeight(Constants.HATCH_LEVEL_2_HEIGHT, true));
		hatchLevel3.whenPressed(new ElevatorAndIntakeHeight(Constants.HATCH_LEVEL_3_HEIGHT, true));
		cargoLevel1.whenPressed(new ElevatorAndIntakeHeight(Constants.CARGO_LEVEL_1_HEIGHT, true));
		cargoLevel2.whenPressed(new ElevatorAndIntakeHeight(Constants.CARGO_LEVEL_2_HEIGHT, true));
		cargoLevel3.whenPressed(new ElevatorAndIntakeHeight(Constants.CARGO_LEVEL_3_HEIGHT, true));
		cargoShipHeight.whenPressed(new ElevatorAndIntakeHeight(Constants.CARGO_SHIP_HEIGHT, false));

		intakeHatch.whenPressed(new IntakeHatch());
		outtakeHatch.whenPressed(new OuttakeHatch(Constants.HATCH_EJECT_RETRACT_TIMEOUT));
		intakeCargo.whenPressed(new CargoCommand(true, Constants.CARGO_INTAKE_INPUT_MAGNITUDE));
		outtakeCargo.whenPressed(new CargoCommand(false, Constants.CARGO_INTAKE_INPUT_MAGNITUDE));
	}
}
