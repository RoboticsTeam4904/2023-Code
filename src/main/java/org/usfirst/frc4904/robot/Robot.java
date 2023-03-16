/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.seenoevil.RobotContainer2;
import org.usfirst.frc4904.robot.seenoevil.RobotContainer2.Component;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;
import org.usfirst.frc4904.standard.humaninput.Operator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends CommandRobotBase {
    private final RobotMap map = new RobotMap();
    private final RobotContainer2 donttouchme = new RobotContainer2(RobotMap.Component.frontLeftWheelTalon, RobotMap.Component.backLeftWheelTalon, RobotMap.Component.frontRightWheelTalon, RobotMap.Component.backRightWheelTalon, RobotMap.Component.navx);
    private final Driver driver = new NathanGain();
    private final Operator operator = new DefaultOperator();


    @Override
    public void initialize() {
        // driverChooser.setDefaultOption(new NathanGain());
        // operatorChooser.setDefaultOption(new DefaultOperator()); 
    }

    @Override
    public void teleopInitialize() {
        // RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(0, 60, 60);
        // RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(() -> 0).schedule();

        driver.bindCommands();
        operator.bindCommands();
    }

    @Override
    public void teleopExecute() {
        // RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setVoltage(2);

        
    }

    @Override
    public void autonomousInitialize() {
        RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 

        final Trajectory backward = donttouchme.getTrajectory("straight_backward");
        // donttouchme.getTrajectory("straight_backward");
        final Trajectory forward = donttouchme.getTrajectory("straight_forward");
        // donttouchme.m_robotDrive.tankDriveVolts(5, 5);
        // var command = new SequentialCommandGroup(donttouchme.getAutonomousCommand(trajectory), donttouchme.getAutonomousCommand(trajectory2));
        var commnand = donttouchme.getAutonomousCommand(backward);
        commnand.schedule();
        // var command2 = donttouchme.getAutonomousCommand(trajectory2);
        // command2.andThen(command).schedule();
        // command.andThen(Commands.runOnce(() -> donttouchme.getAutonomousCommand(trajectory))).schedule();
    }

    @Override
    public void autonomousExecute() {
        SmartDashboard.putString("pose", donttouchme.m_robotDrive.getPose().toString());
    }

    @Override
    public void disabledInitialize() {
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
// RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast); 
//         RobotContainer2.Component.leftATalonFX.neutralOutput();
//         RobotContainer2.Component.leftBTalonFX.neutralOutput();
//         RobotContainer2.Component.rightATalonFX.neutralOutput();
//         RobotContainer2.Component.rightBTalonFX.neutralOutput();        
        
    }

    @Override
    public void testExecute() {
        RobotMap.Component.chassis.testFeedForward(0.5);
        SmartDashboard.putNumber("backward vel ", RobotMap.Component.chassis.leftMotors.leadMotor.getSelectedSensorVelocity(0) /2048 / RobotMap.Metrics.Chassis.GEAR_RATIO * RobotMap.Metrics.Chassis.WHEEL_DIAMETER_METERS*Math.PI);
    }

    @Override
    public void alwaysExecute() {
    }

}

