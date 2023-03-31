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
import org.usfirst.frc4904.robot.seenoevil.DriveSubsystem;
import org.usfirst.frc4904.robot.seenoevil.RobotContainer2;
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
    private final RobotContainer2 donttouchme = new RobotContainer2(RobotMap.Component.frontLeftWheelTalon, RobotMap.Component.backLeftWheelTalon, RobotMap.Component.frontRightWheelTalon, RobotMap.Component.backRightWheelTalon, RobotMap.Component.navx);

    private final Driver driver = new NathanGain();
    private final org.usfirst.frc4904.standard.humaninput.Operator operator = new DefaultOperator();

    @Override
    public void initialize() {
    }

    @Override
    public void teleopInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null) RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null) RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();
        RobotMap.Component.arm.armExtensionSubsystem.motor.setBrakeOnNeutral();

        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setBrakeOnNeutral();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();
        
        /***********************
         * HAZMAT BLOCK START
        *************************/
        // SATURDAY MORNING TEST - can you run drive train in queueline
        
        donttouchme.m_robotDrive.m_leftMotors = null;
        donttouchme.m_robotDrive.m_rightMotors = null;
        donttouchme.m_robotDrive.m_drive = null;
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

        final DoubleSupplier pivot_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) * 50;  
        (new Trigger(() -> pivot_getter.getAsDouble() != 0)).onTrue(
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
        var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);

        var cmd2 = RobotMap.Component.intake.c_holdVoltage(-8);
		cmd2.setName("Intake - manual intake activation");
        var cmdhold = RobotMap.Component.intake.c_holdVoltage(-2).withTimeout(0.5).andThen(RobotMap.Component.intake.c_holdVoltage(-1));

		cmdnull.setName("Intake - deactivated");
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(cmd2);
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(cmdhold);

		// Outtake
		var cmd1 = RobotMap.Component.intake.c_holdVoltage(3);
		cmd1.setName("Intake - manual outtake activation");
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(cmd1);
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(cmdnull);

        		// position + place cube
 		RobotMap.HumanInput.Operator.joystick.button7.onTrue(RobotMap.Component.arm.c_angleCubes(3));
 		RobotMap.HumanInput.Operator.joystick.button9.onTrue(RobotMap.Component.arm.c_angleCubes(2));
 		RobotMap.HumanInput.Operator.joystick.button11.onTrue(RobotMap.Component.arm.c_angleCubes(1));

 		//position cone
 		RobotMap.HumanInput.Operator.joystick.button8.onTrue(RobotMap.Component.arm.placeCones(3));
 		RobotMap.HumanInput.Operator.joystick.button10.onTrue(RobotMap.Component.arm.placeCones(2));
 		RobotMap.HumanInput.Operator.joystick.button12.onTrue(RobotMap.Component.arm.placeCones(1));

 		//shelf intake
 		RobotMap.HumanInput.Operator.joystick.button6.onTrue(RobotMap.Component.arm.c_posIntakeShelf());

 		//ground intake
 		RobotMap.HumanInput.Operator.joystick.button4.onTrue(RobotMap.Component.arm.c_posIntakeGround());
    }

    @Override
    public void teleopExecute() {
        // SmartDashboard.putNumber("Controller out", RobotMap.HumanInput.Driver.xbox.getLeftX());
        // SmartDashboard.putNumber("Controller in trigger", RobotMap.HumanInput.Driver.xbox.getRightTriggerAxis());

        // SmartDashboard.putNumber("left in", driver.getY() + 1 * driver.getTurnSpeed() * 12);
        // SmartDashboard.putNumber("right in", driver.getY() - 1 * driver.getTurnSpeed() * 12);

        
        // SmartDashboard.putNumber("Driver out", driver.getTurnSpeed());
        

        SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        SmartDashboard.putNumber("arm extension length", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());
        SmartDashboard.putNumber("arm pivot angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());

        SmartDashboard.putNumber("Falcon temp",  RobotContainer2.Component.leftATalonFX.getTemperature());


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
        var commnand = donttouchme.getAutonomousCommand(donttouchme.getTrajectory("straight_forward"));
        commnand.schedule();
    }

    @Override
    public void autonomousExecute() {
        // TODO remove logging
        

        SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        SmartDashboard.putString("pose string", donttouchme.m_robotDrive.getPose().toString());
        SmartDashboard.putNumber("pose x", donttouchme.m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("pose y", donttouchme.m_robotDrive.getPose().getY());
        SmartDashboard.putNumber("pose heading", donttouchme.m_robotDrive.getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        SmartDashboard.putNumber("armV extension length", RobotMap.Component.arm.armExtensionSubsystem.getCurrentExtensionLength());
        SmartDashboard.putNumber("arm pivot angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());

        SmartDashboard.putNumber("Falcon temp",  RobotContainer2.Component.leftATalonFX.getTemperature());
        
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


       
    }

    @Override
    public void alwaysExecute() {
        // SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        // SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());


        
    }

}

