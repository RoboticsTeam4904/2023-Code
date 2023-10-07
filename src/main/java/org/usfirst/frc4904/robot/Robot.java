/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;

import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;
import org.usfirst.frc4904.robot.humaninterface.operators.DefaultOperator;
import org.usfirst.frc4904.robot.seenoevil.DriveSubsystem;
import org.usfirst.frc4904.robot.seenoevil.RobotContainer2;
import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.humaninput.Driver;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.usfirst.frc4904.robot.Utils.nameCommand;


public class Robot extends CommandRobotBase {
    private final RobotMap map = new RobotMap();
    private final RobotContainer2 donttouchme = new RobotContainer2(RobotMap.Component.frontLeftWheelTalon, RobotMap.Component.backLeftWheelTalon, RobotMap.Component.frontRightWheelTalon, RobotMap.Component.backRightWheelTalon, RobotMap.Component.navx);
    private final NathanGain nathangain = new NathanGain();
    private final Driver driver = new NathanGain();
    private final org.usfirst.frc4904.standard.humaninput.Operator operator = new DefaultOperator();

    protected double scaleGain(double input, double gain, double exp) {
		return Math.pow(Math.abs(input), exp) * gain * Math.signum(input);
	}

    @Override
    public void initialize() {
    }

    @Override
    public void teleopInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null) RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null) RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.brake();

        RobotMap.Component.arm.armExtensionSubsystem.motor.brake();

        driver.bindCommands();
        operator.bindCommands();
        
        /***********************
         * HAZMAT BLOCK START
        *************************/
        // SATURDAY MORNING TEST - can you run drive train in queueline
        
        // donttouchme.m_robotDrive.m_leftMotors = null;
        // donttouchme.m_robotDrive.m_rightMotors = null;
        // donttouchme.m_robotDrive.m_drive = null;
        DriveSubsystem.skuffedaf_teleop_initialized = true;
        
        
        // donttouchme.m_robotDrive.m_leftEncoder = null;
        // donttouchme.m_robotDrive.m_rightEncoder = null;
        
        // RobotContainer2.Component.leftATalonFX = null;
        // RobotContainer2.Component.leftBTalonFX = null;

        // RobotContainer2.Component.rightATalonFX = null;
        // RobotContainer2.Component.rightBTalonFX = null;

        /***********************
         * HAZMAT BLOCK END
        *************************/


        final double TURN_MULTIPLIER = 2;
        RobotMap.Component.chassis.setDefaultCommand(
            nameCommand("chassis - Teleop_Default - c_controlWheelVoltages", 
                RobotMap.Component.chassis.c_controlWheelVoltages(
                    () -> new DifferentialDriveWheelVoltages(
                        (driver.getY() + TURN_MULTIPLIER * driver.getTurnSpeed()) * 12,
                        (driver.getY() - TURN_MULTIPLIER * driver.getTurnSpeed()) * 12
        ))));

        // RobotMap.Component.arm.setDefaultCommand(nameCommand("arm - default command",
        //     RobotMap.Component.arm.c_posReturnToHomeUp(NathanGain.isFlippy)
        // ));

        final DoubleSupplier pivot_getter = () -> scaleGain(RobotMap.HumanInput.Operator.joystick.getAxis(1),60, 1);  
        (new Trigger(() -> pivot_getter.getAsDouble() != 0)).onTrue(
            nameCommand("arm - teleop - armPivot operator override",
                RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(pivot_getter::getAsDouble)
            )
        );

        // Intake
		// FIXME: use nameCommand to make it cleaner with expresions (no varibales) 
        // var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);

    }

    @Override
    public void teleopExecute() {
        // SmartDashboard.putNumber("Controller out", RobotMap.HumanInput.Driver.xbox.getLeftX());
        // SmartDashboard.putNumber("Controller in trigger", RobotMap.HumanInput.Driver.xbox.getRightTriggerAxis());

        // SmartDashboard.putNumber("left in", driver.getY() + 1 * driver.getTurnSpeed() * 12);
        // SmartDashboard.putNumber("right in", driver.getY() - 1 * driver.getTurnSpeed() * 12);

        
        // SmartDashboard.putNumber("Driver out", driver.getTurnSpeed());
        

        // SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        // if (RobotMap.HumanInput.Operator.joystick.getPOV() != 0) {
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        SmartDashboard.putNumber("arm extension length", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());
        SmartDashboard.putNumber("zeroing", RobotMap.Component.arm.armExtensionSubsystem.motor.getSensorPositionRotations());
        SmartDashboard.putNumber("arm pivot angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        //}
        // SmartDashboard.putNumber("Falcon temp",  RobotContainer2.Component.leftATalonFX.getTemperature());

        SmartDashboard.putData(RobotMap.Component.arm.armPivotSubsystem);
        SmartDashboard.putData(RobotMap.Component.arm.armExtensionSubsystem);


    }

    @Override
    public void autonomousInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null)  RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null)  RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 

        
        // if (RobotContainer2.Component.leftATalonFX != null)  { RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);  RobotContainer2.Component.leftATalonFX.neutralOutput(); }
        // if (RobotContainer2.Component.leftBTalonFX != null)  { RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);  RobotContainer2.Component.leftBTalonFX.neutralOutput(); }
        // if (RobotContainer2.Component.rightATalonFX != null) { RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast); RobotContainer2.Component.rightATalonFX.neutralOutput(); } 
        // if (RobotContainer2.Component.rightBTalonFX != null) { RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast); RobotContainer2.Component.rightBTalonFX.neutralOutput(); } 

        // hold arm pose
        // RobotMap.Component.arm.c_holdArmPose(0, 0.5).schedule();


        // arm pose individual tests
        // RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(6, 150, 200).getFirst().schedule();
        // // RobotMap.Component.arm.c_holdArmPose(6, 0.5);
        // RobotMap.Component.arm.armExtensionSubsystem.c_holdExtension(0.5, 1, 2).getFirst().schedule();



        // SATURDAY MORNING TEST: is the cube shooter auton gonna work
        // var commnand = donttouchme.balanceAutonAndShootCube(donttouchme.m_robotDrive::getWheelSpeeds, donttouchme.m_robotDrive::tankDriveVolts);
        // var commnand = donttouchme.balanceAutonAndShootCube(donttouchme.m_robotDrive::getWheelSpeeds, donttouchme.m_robotDrive::tankDriveVolts);
        
        var commnand = donttouchme.simpleAuton(); //for calgames; just place a cone and leave the zone
        commnand.schedule();
    }

    @Override
    public void autonomousExecute() {
        // TODO remove logging
        

        // SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        SmartDashboard.putString("pose string", donttouchme.m_robotDrive.getPose().toString());
        SmartDashboard.putNumber("pose x", donttouchme.m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("pose y", donttouchme.m_robotDrive.getPose().getY());
        SmartDashboard.putNumber("pose heading", donttouchme.m_robotDrive.getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        SmartDashboard.putNumber("armV extension length", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());
        SmartDashboard.putNumber("arm pivot angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());

        // SmartDashboard.putNumber("Falcon temp",  RobotContainer2.Component.leftATalonFX.getTemperature());
        
        // SmartDashboard.putData(RobotMap.Component.arm.armPivotSubsystem);
        // SmartDashboard.putData(RobotMap.Component.arm.armExtensionSubsystem);
        // SmartDashboard.putData(RobotMap.Component.arm);
        // SmartDashboard.putData(donttouchme.m_robotDrive);
    }

    @Override
    public void disabledInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null)  RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null)  RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();
        RobotMap.Component.arm.armExtensionSubsystem.motor.setBrakeOnNeutral();

        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setBrakeOnNeutral();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();

        RobotMap.Component.arm.armExtensionSubsystem.motor.setBrakeOnNeutral();
        RobotMap.Component.arm.armExtensionSubsystem.motor.neutralOutput();
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
                if (RobotContainer2.Component.leftATalonFX != null)  { RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast);  RobotContainer2.Component.leftATalonFX.neutralOutput(); }
                if (RobotContainer2.Component.leftBTalonFX != null)  { RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast);  RobotContainer2.Component.leftBTalonFX.neutralOutput(); }
                if (RobotContainer2.Component.rightATalonFX != null) { RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast); RobotContainer2.Component.rightATalonFX.neutralOutput(); } 
                if (RobotContainer2.Component.rightBTalonFX != null) { RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast); RobotContainer2.Component.rightBTalonFX.neutralOutput(); } 
        RobotMap.Component.arm.armPivotSubsystem.initializeEncoderPositions();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setCoastOnNeutral();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();

        RobotMap.Component.arm.armExtensionSubsystem.motor.setCoastOnNeutral();
        RobotMap.Component.arm.armExtensionSubsystem.motor.neutralOutput();
    }

    @Override
    public void testExecute() {
        RobotMap.Component.arm.armExtensionSubsystem.initializeEncoderPositions(0);
        RobotMap.Component.intake.c_holdVoltage(0);

       
    }

    @Override
    public void alwaysExecute() {
        SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        SmartDashboard.putNumber("arm extension length", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());
        SmartDashboard.putNumber("intake speed", RobotMap.HumanInput.Operator.joystick.getAxis(3));


        // SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
    }

}

