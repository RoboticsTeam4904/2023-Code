package org.usfirst.frc4904.robot.subsystems;

import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static int DEFAULT_INTAKE_VOLTS = 3;
    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;
    public Intake (CANSparkMax leftMotor, CANSparkMax rightMotor){ //motors has leftmotor and rightmotot
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        // FIXME: actual current limits (55 is way high)
        // FIXME: actual current limits (55 is way high)
        rightMotor.setSmartCurrentLimit(55);
        leftMotor.setSmartCurrentLimit(55);
    }
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(-voltage);
    }

    public Command c_neutralOutput() {
        return Commands.runOnce(() -> {
            leftMotor.stopMotor();
            rightMotor.stopMotor();;
        });
    }
    public Command c_holdVoltage(double voltage) {
        return Commands.run(() -> {
            setVoltage(voltage);
        });
    }
    public Command c_startIntake() {
        return c_holdVoltage(-8);
    }
    public Command c_holdItem() {
        return c_holdVoltage(-2).withTimeout(0.5).andThen(c_holdVoltage(-1.6));
    }

    public Command c_holdVoltageDefault() {
        var cmd = Commands.run(() -> setVoltage(DEFAULT_INTAKE_VOLTS));
        cmd.setName("Intake - c_holdVoltageDefault");
        return cmd;
    }
}