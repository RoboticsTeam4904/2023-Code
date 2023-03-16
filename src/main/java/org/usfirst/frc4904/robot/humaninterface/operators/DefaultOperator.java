package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {
        // RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) / 4
        // ).schedule();
        // RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(
        //     () -> RobotMap.HumanInput.Operator.joystick.getAxis(2) * 0.5
        // ).schedule();
	}
}
