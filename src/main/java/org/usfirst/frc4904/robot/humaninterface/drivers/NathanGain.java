package org.usfirst.frc4904.robot.humaninterface.drivers;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class NathanGain extends Driver {
	public static final double SPEED_GAIN = 1;
	public static final double SPEED_EXP = 2;
	public static final double TURN_GAIN = 0.6;
	public static final double TURN_EXP = 1;
	public static final double Y_SPEED_SCALE = 1;
	public static final double TURN_SPEED_SCALE = 1;

	public NathanGain() {
		super("NathanGain");
	}

	protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

	@Override
	public void bindCommands() {
		// RobotMap.Component.chassis.setDefaultCommand(
		// 	RobotMap.Component.chassis.c_controlChassisVelocity(
		// 		() -> new ChassisSpeeds(getY(), getX(), getTurnSpeed())
		// 	)
		// );
	}

	@Override
	public double getX() {
		return 0;
	}

	@Override
	public double getY() {
		//TODO: new getx is 0 to 1, old may have been -1 to 1 -- fix is simple if we need it
		double rawSpeed = RobotMap.HumanInput.Driver.xbox.getRightTriggerAxis() - RobotMap.HumanInput.Driver.xbox.getLeftTriggerAxis();
		double speed = scaleGain(rawSpeed, NathanGain.SPEED_GAIN, NathanGain.SPEED_EXP) * NathanGain.Y_SPEED_SCALE;
		double precisionDrive = scaleGain(RobotMap.HumanInput.Driver.xbox.getLeftY(), 0.08, 1.2);
		double operatorDrive = scaleGain(-RobotMap.HumanInput.Operator.joystick.getAxis(1), 0.1, 1.2);
		return speed + precisionDrive + operatorDrive;
	}

	@Override
	public double getTurnSpeed() {
		double rawTurnSpeed = RobotMap.HumanInput.Driver.xbox.getLeftX();
		double turnSpeed = scaleGain(rawTurnSpeed, NathanGain.TURN_GAIN, NathanGain.TURN_EXP) * NathanGain.TURN_SPEED_SCALE;
		double precisionTurnSpeed = scaleGain(RobotMap.HumanInput.Driver.xbox.getRightX(), 0.08, 1.2);
		double operatorControlTurnSpeed = scaleGain(RobotMap.HumanInput.Operator.joystick.getAxis(0), 0.2, 1.5);
		return turnSpeed + precisionTurnSpeed + operatorControlTurnSpeed;
	}
}
