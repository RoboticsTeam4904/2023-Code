/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


// TODO implement test and sim in CommandRobotBase
public class Robot extends CommandRobotBase {
    private final CANTalonFX lefta = new CANTalonFX(1, InvertType.None);
    private final CANTalonFX leftb = new CANTalonFX(2, InvertType.None);
    private final TalonMotorSubsystem leftMotors = new TalonMotorSubsystem("yes", NeutralMode.Brake, 10, lefta, leftb);

    // private RobotMap map = new RobotMap();

    @Override
    public void initialize() {
        // driverChooser.setDefaultOption(new NathanGain());
        // operatorChooser.setDefaultOption(new DefaultOperator());
        // autoChooser.setDefaultOption(RobotMap.Autonomous.autonCommand);  // zach is worried that this will get misclicked -> screw us
    }

    @Override
    public void teleopInitialize() {
        // var controller = RobotMap.HumanInput.Driver.xbox;
        // var maxSpeed = 12;
        // RobotMap.Component.chassis.leftMotors.setBrakeOnNeutral();
        // RobotMap.Component.chassis.rightMotors.setBrakeOnNeutral();

        // // RobotMap.Component.chassis.setDefaultCommand(RobotMap.Component.chassis.c_controlChassisSpeedAndTurn(() -> new Pair<Double, Double>(Robot.drivingConfig.getX(), Robot.drivingConfig.getTurnSpeed())));
        // RobotMap.Component.chassis.c_controlWheelVoltages(() -> new DifferentialDriveWheelVoltages(
        //     (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() + 1*controller.getLeftX())*maxSpeed,
        //     (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() - 1*controller.getLeftX())*maxSpeed
        // )).schedule();
    }

    @Override
    public void teleopExecute() {
        // System.out.println(RobotMap.Component.chassis.leftMotors.leadMotor.get());
    }

    @Override
    public void autonomousInitialize() {
        // TODO: remove; for testing only
        // RobotMap.Component.chassis.leftMotors.setBrakeOnNeutral();
        // RobotMap.Component.chassis.rightMotors.setBrakeOnNeutral();

    }

    @Override
    public void autonomousExecute() {
        leftMotors.set(-0.3);

    }

    @Override
    public void disabledInitialize() {
        // TODO: remove; for testing only
        new Timer().schedule(new TimerTask() { // https://stackoverflow.com/a/56225206/10372825
            public void run() {
                RobotMap.Component.chassis.leftMotors.coast();
                RobotMap.Component.chassis.rightMotors.coast();
            }
        }, 2 * 1000L);  // coast motors after 2 seconds
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
        RobotMap.Component.chassis.setChassisVelocity(new ChassisSpeeds(0, 0, 1));
        // RobotMap.Component.chassis.setWheelVoltages(new DifferentialDriveWheelVoltages(3, 3));
        //robot jerks around when trying to go forward
        //robot stopped responding even w/ green code light
    }

    @Override
    public void testExecute() {
    }

    @Override
    public void alwaysExecute() {
    }

}
