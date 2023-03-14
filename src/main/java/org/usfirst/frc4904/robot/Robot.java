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

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// TODO implement test and sim in CommandRobotBase
public class Robot extends CommandRobotBase {
    private static RobotMap map = new RobotMap();

    @Override
    public void initialize() {
        driverChooser.setDefaultOption(new NathanGain());
        operatorChooser.setDefaultOption(new DefaultOperator());
        // autoChooser.setDefaultOption(RobotMap.Autonomous.autonCommand);  // zach is worried that this will get misclicked -> screw us
    }

    @Override
    public void teleopInitialize() {
    }

    @Override
    public void teleopExecute() {
    }

    @Override
    public void autonomousInitialize() {
        // TODO: remove; for testing only
        RobotMap.Component.chassis.leftMotors.setBrakeOnNeutral();
        RobotMap.Component.chassis.rightMotors.setBrakeOnNeutral();
    }

    @Override
    public void autonomousExecute() {
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
        try {
            SmartDashboard.putBoolean("extension Fwd limit pressed", RobotMap.Component.arm.armExtensionSubsystem.getMotor().leadMotor.isFwdLimitSwitchPressed());
            SmartDashboard.putBoolean("extension Rev limit pressed", RobotMap.Component.arm.armExtensionSubsystem.getMotor().leadMotor.isRevLimitSwitchPressed());
            SmartDashboard.putBoolean("pivot Fwd limit pressed", RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.leadMotor.isFwdLimitSwitchPressed());
            SmartDashboard.putBoolean("pivot Rev limit pressed", RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.leadMotor.isRevLimitSwitchPressed());
        } catch (IllegalAccessException e) {
            //cry
            // this should never happen because we don't use spark maxes AND we won't be declaring spark maxes and trying to low-level-read their limit switch status w/o constructing with the correct limit switch type
        }
    }

}
