package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * The subsystem for controlling all game piece manipulation.
 */
public class Superstructure extends Subsystem {
    private final Solenoid m_hatchFingers, m_hatchEjectors, m_intakeHeight;
    private final TalonSRX m_cargoIntake;

    public Superstructure() {
        m_hatchFingers = new Solenoid(Constants.PCMIDs.HATCH_FINGERS);
        m_hatchEjectors = new Solenoid(Constants.PCMIDs.HATCH_EJECTORS);
        m_cargoIntake = new TalonSRX(Constants.CANIDs.INTAKE);
        m_intakeHeight = new Solenoid(Constants.PCMIDs.INTAKE_HEIGHT);
    }

    public void useHatchFingers(boolean releasing) {
        m_hatchFingers.set(!releasing);
    }

    public void useHatchEjectors(boolean extending) {
        m_hatchEjectors.set(extending);
    }

    public void setIntakeHeight(boolean isHigh) {
        m_intakeHeight.set(!isHigh);
    }

    public void setCargoIntakePercentOutput(double percentOutput) {
        m_cargoIntake.set(ControlMode.PercentOutput, percentOutput);
    }
    
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
