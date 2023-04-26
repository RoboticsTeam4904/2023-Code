package org.usfirst.frc4904.robot.humaninterface.operators;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.standard.commands.TriggerCommandFactory;
import org.usfirst.frc4904.standard.humaninput.Operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DefaultOperator extends Operator {
	public DefaultOperator() {
		super("DefaultOperator");
	}

	public DefaultOperator(String name) {
		super(name);
	}

	@Override
	public void bindCommands() {

		final DoubleSupplier pivot_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(1);
		
		// might not work
        (new Trigger(() -> pivot_getter.getAsDouble() != 0)).whileTrue( 
			new RunCommand(() -> RobotMap.Component.arm.armPivotSubsystem.setVelocity(pivot_getter.getAsDouble()), RobotMap.Component.arm.armPivotSubsystem)
		);
           
		
			var joystick = RobotMap.HumanInput.Operator.joystick;
		
			// manual extension and retraction
			joystick.button3.onTrue(new InstantCommand(() -> RobotMap.Component.arm.armExtensionSubsystem.setVelocity(-0.45)));
			joystick.button3.onFalse(new InstantCommand(() -> RobotMap.Component.arm.armExtensionSubsystem.setVelocity(0)));
			joystick.button5.onTrue(new InstantCommand(() -> RobotMap.Component.arm.armExtensionSubsystem.setVelocity(0.45)));
			joystick.button5.onFalse(new InstantCommand(() -> RobotMap.Component.arm.armExtensionSubsystem.setVelocity(0)));
	
			// Intake
			// FIXME: use nameCommand to make it cleaner with expresions (no varibales)
			var zeroIntake = RobotMap.Component.intake.c_holdVoltage(0);
			var runOuttake = RobotMap.Component.intake.c_holdVoltage(3);
	
			// intake
			joystick.button2.onTrue(RobotMap.Component.intake.c_startIntake());
			joystick.button2.onFalse(RobotMap.Component.intake.c_holdItem());
	
			// outtake
			joystick.button1.onTrue(runOuttake);
			joystick.button1.onFalse(zeroIntake);
	
	
			// position + place cube
			joystick.button7.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_shootCubes(3)));
			joystick.button9.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_shootCubes(2)));
	
			// position cone
			joystick.button8.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_shootCones(3)));
			joystick.button10.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_shootCones(2)));
	
	
			// intake positions
			joystick.button6.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posIntakeShelf()));
			joystick.button4.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posIntakeFloor()));
	
			// stow positions
			joystick.button11.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posReturnToHomeDown()));
			joystick.button12.onTrue(new TriggerCommandFactory(() -> RobotMap.Component.arm.c_posReturnToHomeUp()));
	}
	
}
