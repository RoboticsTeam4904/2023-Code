package org.usfirst.frc4904.robot.humaninterface.drivers;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Driver;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class NathanGain extends Driver {
	public static boolean isFlippy = false;
	
	public static final double SPEED_EXP = 2;
	public static final double Y_SPEED_SCALE = 1;

	public static final double TURN_EXP = 4;
	public static final double TURN_SPEED_SCALE = 1;


	public static final double NORMAL_SPEED_GAIN = 0.5; // TODO TUNE
	public static final double NORMAL_TURN_GAIN = 0.25;

	public static final double PRECISE_SPEED_SCALE = 0.2;	
	public static final double PRECISE_TURN_SCALE = 0.1;

	// public static final double SPEEDY_TURN_GAIN = 0.7;

	public static double precisionScaleY = NORMAL_SPEED_GAIN;
	public static double precisionScaleTurn = NORMAL_TURN_GAIN;

	public NathanGain() {
		super("NathanGain");
	}

	protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

	@Override
	public void bindCommands() {
		RobotMap.HumanInput.Driver.throttleJoystick.button1.onTrue(new InstantCommand(
			() -> {
				NathanGain.precisionScaleY = PRECISE_SPEED_SCALE;
				NathanGain.precisionScaleTurn = PRECISE_TURN_SCALE;
			}
		));
		RobotMap.HumanInput.Driver.throttleJoystick.button1.onFalse(new InstantCommand((	
			) -> {
				NathanGain.precisionScaleY = NORMAL_SPEED_GAIN;
				NathanGain.precisionScaleTurn = NORMAL_TURN_GAIN;
			}
		));
		
		RobotMap.HumanInput.Driver.throttleJoystick.button1.onTrue(new InstantCommand(() -> NathanGain.precisionScaleY = 1));
		RobotMap.HumanInput.Driver.throttleJoystick.button1.onFalse(new InstantCommand(() -> NathanGain.precisionScaleY = NORMAL_SPEED_GAIN));

		RobotMap.HumanInput.Driver.throttleJoystick.button2.onTrue(new InstantCommand(() -> NathanGain.precisionScaleY = 1));
		RobotMap.HumanInput.Driver.throttleJoystick.button2.onFalse(new InstantCommand(() -> NathanGain.precisionScaleY = NORMAL_SPEED_GAIN));

		RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(new InstantCommand(() -> NathanGain.precisionScaleY = 1));
		RobotMap.HumanInput.Driver.turnJoystick.button1.onFalse(new InstantCommand(() -> NathanGain.precisionScaleY = NORMAL_SPEED_GAIN));

		RobotMap.HumanInput.Driver.turnJoystick.button2.onTrue(new InstantCommand(() -> NathanGain.precisionScaleY = 1));
		RobotMap.HumanInput.Driver.turnJoystick.button1.onFalse(new InstantCommand(() -> NathanGain.precisionScaleY = NORMAL_SPEED_GAIN));


	}

	@Override
	public double getX() {
		return 0;
	}

	@Override
	public double getY() {
		double rawSpeed = RobotMap.HumanInput.Driver.throttleJoystick.getY();
		double speed;
		
		speed = scaleGain(rawSpeed, NathanGain.precisionScaleY, NathanGain.SPEED_EXP) * NathanGain.Y_SPEED_SCALE;
	

		// double precisionDrive = scaleGain(RobotMap.HumanInput.Driver.xbox.getLeftY(), 0.08, 1.2);
		// double operatorDrive = scaleGain(-RobotMap.HumanInput.Operator.joystick.getAxis(1), 0.1, 1.2);
		
		if (NathanGain.isFlippy) return (speed) * -1;
		return speed;// + precisionDrive;
	}

	@Override
	public double getTurnSpeed() {
		double rawTurnSpeed = RobotMap.HumanInput.Driver.turnJoystick.getX();
		double turnSpeed = scaleGain(rawTurnSpeed, NathanGain.precisionScaleTurn, NathanGain.TURN_EXP) * NathanGain.TURN_SPEED_SCALE;
		// double precisionTurnSpeed = scaleGain(RobotMap.HumanInput.Driver.xbox.getRightX(), 0.08, 1.2);
		// double operatorControlTurnSpeed = scaleGain(RobotMap.HumanInput.Operator.joystick.getAxis(0), 0.2, 1.5);
		
		if (NathanGain.isFlippy) return (turnSpeed);
		return turnSpeed*-1;// + precisionTurnSpeed; //is mounted inverted so we need to revert by default
	}
}
