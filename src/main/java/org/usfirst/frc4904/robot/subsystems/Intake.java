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
        leftMotor.setSmartCurrentLimit(55);
        rightMotor.setSmartCurrentLimit(55);
    }
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(-1.3*voltage);
    }
    public Command c_holdVoltage(double voltage) {
        var cmd = this.run(() -> {
            setVoltage(voltage);
        });
        return cmd;
    }

    public Command c_holdVoltageDefault() {
        var cmd = Commands.run(() -> setVoltage(DEFAULT_INTAKE_VOLTS));
        cmd.setName("Intake - c_holdVoltageDefault");
        return cmd;
    }
}