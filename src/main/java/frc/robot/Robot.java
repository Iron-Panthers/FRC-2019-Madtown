/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
	private static final int deviceID = 12;
	private static final int follower2ID = 11;
	private static final int follower3ID = 10;
	private CANSparkMax m_motor;
	private CANSparkMax follower2;
	private CANSparkMax follower3;
	private CANPIDController m_pidController;
	private CANEncoder m_encoder;
	private CANDigitalInput topLimit, bottomLimit;
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
	public static final double topEncoderPosition = 0.0;

	private static Joystick stick1;
	private static JoystickButton manualElevator;
	private static JoystickButton zeroElevator;

	@Override
	public void robotInit() {
		stick1 = new Joystick(0);
		manualElevator = new JoystickButton(stick1, 1);
		zeroElevator = new JoystickButton(stick1, 2);
		// initialize motor
		m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
		follower2 = new CANSparkMax(follower2ID, MotorType.kBrushless);
		follower3 = new CANSparkMax(follower3ID, MotorType.kBrushless);
		/**
		 * The restoreFactoryDefaults method can be used to reset the configuration
		 * parameters in the SPARK MAX to their factory default state. If no argument is
		 * passed, these parameters will not persist between power cycles
		 */
		m_motor.restoreFactoryDefaults();
		follower2.restoreFactoryDefaults();
		follower3.restoreFactoryDefaults();
		follower2.follow(m_motor);
		follower3.follow(m_motor);
		/**
		 * In order to use PID functionality for a controller, a CANPIDController object
		 * is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		m_pidController = m_motor.getPIDController();

		// Encoder object created to display position values
		m_encoder = m_motor.getEncoder();
		topLimit = m_motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		bottomLimit = m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
		topLimit.enableLimitSwitch(true);
		bottomLimit.enableLimitSwitch(true);

		// PID coefficients
		kP = 0.1;
		kI = 0;
		kD = 0;
		kIz = 0;
		kFF = 0;
		kMaxOutput = 1;
		kMinOutput = -1;

		// set PID coefficients
		m_pidController.setP(kP);
		m_pidController.setI(kI);
		m_pidController.setD(kD);
		m_pidController.setIZone(kIz);
		m_pidController.setFF(kFF);
		m_pidController.setOutputRange(kMinOutput, kMaxOutput);

		// display PID coefficients on SmartDashboard
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Zone", kIz);
		SmartDashboard.putNumber("Feed Forward", kFF);
		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);
		SmartDashboard.putNumber("Set Rotations", 0);
	}

	@Override
	public void teleopPeriodic() {
		// 1. Manual control, zero at bottom, find distance to top
		// 2. PID control, go to a rotation between the top and bottom, use to tune P and F
		if (topLimit.get()) {
			System.out.println("Top Limit Encoder Position: " + m_encoder.getPosition());
		}
		if (bottomLimit.get()) {
			System.out.println("Bottom Limit Encoder Position: " + m_encoder.getPosition());
		}
		if (zeroElevator.get()) {
			m_encoder.setPosition(0);
		}
		if (manualElevator.get()) {
			m_motor.set(stick1.getY());
			return;
		}
		// read PID coefficients from SmartDashboard
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iz = SmartDashboard.getNumber("I Zone", 0);
		double ff = SmartDashboard.getNumber("Feed Forward", 0);
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);
		double rotations = SmartDashboard.getNumber("Set Rotations", 0);

		// if PID coefficients on SmartDashboard have changed, write new values to
		// controller
		if ((p != kP)) {
			m_pidController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			m_pidController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			m_pidController.setD(d);
			kD = d;
		}
		if ((iz != kIz)) {
			m_pidController.setIZone(iz);
			kIz = iz;
		}
		if ((ff != kFF)) {
			m_pidController.setFF(ff);
			kFF = ff;
		}
		if ((max != kMaxOutput) || (min != kMinOutput)) {
			m_pidController.setOutputRange(min, max);
			kMinOutput = min;
			kMaxOutput = max;
		}

		/**
		 * PIDController objects are commanded to a set point using the SetReference()
		 * method.
		 * 
		 * The first parameter is the value of the set point, whose units vary depending
		 * on the control type set in the second parameter.
		 * 
		 * The second parameter is the control type can be set to one of four
		 * parameters: com.revrobotics.ControlType.kDutyCycle
		 * com.revrobotics.ControlType.kPosition com.revrobotics.ControlType.kVelocity
		 * com.revrobotics.ControlType.kVoltage
		 */
		m_pidController.setReference(rotations, ControlType.kPosition);

		SmartDashboard.putNumber("SetPoint", rotations);
		SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
	}
}
