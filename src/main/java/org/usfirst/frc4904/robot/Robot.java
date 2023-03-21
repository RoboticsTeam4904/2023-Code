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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        
        // SATURDAY MORNING TEST - can you run drive train in queueline
        // donttouchme.m_robotDrive.m_leftMotors = null;
        // donttouchme.m_robotDrive.m_rightMotors = null;


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

    
        double NORMAL_SPEED_GAIN = 0.5; // TODO TUNE
        double NORMAL_TURN_GAIN = 0.6;
    
        double PRECISE_SPEED_SCALE = 0.4;	
        double PRECISE_TURN_SCALE = 0.3;

        final DoubleSupplier pivot_getter = () -> RobotMap.HumanInput.Operator.joystick.getAxis(1) * 50;  
        (new Trigger(() -> pivot_getter.getAsDouble() != 0)).whileTrue(
            nameCommand("arm - teleop - armPivot operator override",
                RobotMap.Component.arm.armPivotSubsystem.c_controlAngularVelocity(pivot_getter::getAsDouble)
            )
        );		RobotMap.HumanInput.Driver.xbox.leftBumper().onTrue(new InstantCommand(
			() -> {
				NathanGain.precisionScaleY = PRECISE_SPEED_SCALE;
				NathanGain.precisionScaleTurn = PRECISE_TURN_SCALE;
			}
		));
		RobotMap.HumanInput.Driver.xbox.leftBumper().onFalse(new InstantCommand((	
			) -> {
				NathanGain.precisionScaleY = NORMAL_SPEED_GAIN;
				NathanGain.precisionScaleTurn = NORMAL_TURN_GAIN;
			}
		));
		
		RobotMap.HumanInput.Driver.xbox.y().onTrue(new InstantCommand(() -> NathanGain.precisionScaleY = 1));
		RobotMap.HumanInput.Driver.xbox.y().onFalse(new InstantCommand(() -> NathanGain.precisionScaleY = NORMAL_SPEED_GAIN));



        RobotMap.HumanInput.Operator.joystick.button3.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> -0.3));
        RobotMap.HumanInput.Operator.joystick.button3.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));

        RobotMap.HumanInput.Operator.joystick.button5.onTrue(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0.3));
        RobotMap.HumanInput.Operator.joystick.button5.onFalse(RobotMap.Component.arm.armExtensionSubsystem.c_controlVelocity(() -> 0));
		
        // position + place cube
		RobotMap.HumanInput.Operator.joystick.button7.onTrue(RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(22, 150, 200).getFirst());
		RobotMap.HumanInput.Operator.joystick.button9.onTrue(RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(18, 150, 200).getFirst());
		RobotMap.HumanInput.Operator.joystick.button11.onTrue(RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(-35, 150, 200).getFirst());

		

        // Intake
		// FIXME: use nameCommand to make it cleaner with expresions (no varibales) 
		var cmd2 = RobotMap.Component.intake.c_holdVoltage(-8);
		cmd2.setName("Intake - manual intake activation");
		var cmdnull = RobotMap.Component.intake.c_holdVoltage(0);
		cmdnull.setName("Intake - deactivated");
		RobotMap.HumanInput.Operator.joystick.button2.onTrue(cmd2);
        RobotMap.HumanInput.Operator.joystick.button2.onFalse(cmdnull);

		// Outtake
		var cmd1 = RobotMap.Component.intake.c_holdVoltage(6.8);
		cmd1.setName("Intake - manual outtake activation");
		RobotMap.HumanInput.Operator.joystick.button1.onTrue(cmd1);
        RobotMap.HumanInput.Operator.joystick.button1.onFalse(cmdnull);
    }

    @Override
    public void teleopExecute() {
        SmartDashboard.putNumber("Arm angle", RobotMap.Component.arm.armPivotSubsystem.getCurrentAngleDegrees());
        // SmartDashboard.putNumber("Controller out", RobotMap.HumanInput.Driver.xbox.getLeftX());
        // SmartDashboard.putNumber("Controller in trigger", RobotMap.HumanInput.Driver.xbox.getRightTriggerAxis());

        // SmartDashboard.putNumber("left in", driver.getY() + 1 * driver.getTurnSpeed() * 12);
        // SmartDashboard.putNumber("right in", driver.getY() - 1 * driver.getTurnSpeed() * 12);

        
        // SmartDashboard.putNumber("Driver out", driver.getTurnSpeed());
        

        SmartDashboard.putBoolean("isFlipped - IMPORTANT", NathanGain.isFlippy);
        SmartDashboard.putNumber("gyroooo", RobotMap.Component.navx.getAngle());
        // SmartDashboard.putNumber("Falcon temp",  RobotContainer2.Component.leftATalonFX.temp());


    }

    @Override
    public void autonomousInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null)  RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null)  RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 

        // SATURDAY MORNING TEST: is the cube shooter auton gonna work
        var commnand = donttouchme.balanceAutonAndShootCube(donttouchme.m_robotDrive::getWheelSpeeds, donttouchme.m_robotDrive::tankDriveVolts);
        commnand.schedule();
    }

    @Override
    public void autonomousExecute() {
    }

    @Override
    public void disabledInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null)  RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.leftBTalonFX != null)  RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Brake); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Brake); 

        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setBrakeOnNeutral();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();
    }

    @Override
    public void disabledExecute() {
    }

    @Override
    public void testInitialize() {
        if (RobotContainer2.Component.leftATalonFX != null)  RobotContainer2.Component.leftATalonFX.setNeutralMode(NeutralMode.Coast); 
        if (RobotContainer2.Component.leftBTalonFX != null)  RobotContainer2.Component.leftBTalonFX.setNeutralMode(NeutralMode.Coast); 
        if (RobotContainer2.Component.rightATalonFX != null) RobotContainer2.Component.rightATalonFX.setNeutralMode(NeutralMode.Coast); 
        if (RobotContainer2.Component.rightBTalonFX != null) RobotContainer2.Component.rightBTalonFX.setNeutralMode(NeutralMode.Coast); 
        RobotMap.Component.arm.armPivotSubsystem.initializeEncoderPositions();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.setCoastOnNeutral();
        RobotMap.Component.arm.armPivotSubsystem.armMotorGroup.neutralOutput();
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

