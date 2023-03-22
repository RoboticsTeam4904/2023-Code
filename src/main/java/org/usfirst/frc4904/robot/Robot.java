/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.subsystems.arm.ArmPivotSubsystem;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.usfirst.frc4904.robot.Utils.nameCommand;


public class Robot extends CommandRobotBase {
    private final RobotMap map = new RobotMap();

    private final Driver driver = new NathanGain();
    private final org.usfirst.frc4904.standard.humaninput.Operator operator = new DefaultOperator();

    @Override
    public void initialize() {
    }

    @Override
    public void teleopInitialize() {
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Brake); 
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        
        // RobotMap.Component.arm.setDefaultCommand(nameCommand("arm - default command",
        //     RobotMap.Component.arm.c_posReturnToHomeUp(NathanGain.isFlippy)
        // ));

        final DoubleSupplier pivot_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) * 40;  
        (new Trigger(() -> pivot_getter.getAsDouble() != 0)).whileTrue(
            nameCommand("arm - teleop - armPivot operator override",
                RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(pivot_getter::getAsDouble)
            )
        );

        RobotMap.HumanInput.Operator.joystick.button3.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> -0.3));
        RobotMap.HumanInput.Operator.joystick.button3.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

        RobotMap.HumanInput.Operator.joystick.button5.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0.3));
        RobotMap.HumanInput.Operator.joystick.button5.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));


        // Intake
		// FIXME: use nameCommand to make it cleaner with expresions (no varibales) 
		var cmd2 = RobotMap.Component.intake.c_holdVoltage(-8);
		cmd2.setName("Intake - manual intake activation");
		var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);
		cmdnull.setName("Intake - deactivated");
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(cmd2);
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(cmdnull);

		// Outtake
		var cmd1 = RobotMap.Component.intake.c_holdVoltage(3);
		cmd1.setName("Intake - manual outtake activation");
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(cmd1);
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(cmdnull);
    }

    @Override
    public void teleopExecute() {
        RobotMap.Component.chassis.arcadeDrive(-driver.getY(), driver.getX(), true);
    }

    @Override
    public void autonomousInitialize() {
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void autonomousExecute() {
    }

    @Override
    public void disabledInitialize() {
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Coast);

        RobotMap.Component.pivotMotorLeft.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.pivotMotorLeft.neutralOutput();

        RobotMap.Component.pivotMotorRight.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.pivotMotorRight.neutralOutput();
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Coast);

        RobotMap.Component.arm.armPivotSubsystem.initializeEncoderPositions();

        RobotMap.Component.pivotMotorLeft.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.pivotMotorLeft.neutralOutput();

        RobotMap.Component.pivotMotorRight.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.pivotMotorRight.neutralOutput();
    }

    @Override
    public void testExecute() {
        RobotMap.Component.arm.armExtensionSubsystem.initializeEncoderPositions(0);
    }

    @Override
    public void alwaysExecute() {
        SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
    }

}

