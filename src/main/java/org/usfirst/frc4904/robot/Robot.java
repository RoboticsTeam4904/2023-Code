/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.subsystems.arm.ArmPivotSubsystem;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        driver.bindCommands();
        operator.bindCommands();
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Brake); 
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Brake);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Brake);
        
        // RobotMap.Component.arm.setDefaultCommand(nameCommand("arm - default command",
        //     RobotMap.Component.arm.c_posReturnToHomeUp(NathanGain.isFlippy)
        // ));



    }

    @Override
    public void teleopExecute() {
        //TODO: check chassis inversion?
        RobotMap.Component.chassis.arcadeDrive(driver.getY(), driver.getTurnSpeed(), true);
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
        System.out.println("test enabled");
        RobotMap.Component.frontLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.frontRightWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backLeftWheelTalon.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.backRightWheelTalon.setNeutralMode(NeutralMode.Coast);

        RobotMap.Component.arm.armPivotSubsystem.initializeEncoderPositions();

        RobotMap.Component.pivotMotorLeft.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.pivotMotorLeft.neutralOutput();

        RobotMap.Component.pivotMotorRight.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.pivotMotorRight.neutralOutput();

        RobotMap.Component.armExtensionMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.Component.armExtensionMotor.neutralOutput();

        RobotMap.Component.arm.armExtensionSubsystem.initializeEncoderPositions(0);



    }

    @Override
    public void testExecute() {
    }

    @Override
    public void alwaysExecute() {
        SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        SmartDashboard.putNumber("Arm extension", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());

        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());

        SmartDashboard.putNumber("supply current", RobotMap.Component.arm.armPivotSubsystem.getLeftMotor().getSupplyCurrent());
        SmartDashboard.putNumber("stator current", RobotMap.Component.arm.armPivotSubsystem.getLeftMotor().getStatorCurrent());

    }


}

