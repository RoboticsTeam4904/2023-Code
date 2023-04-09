// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmExtensionFeedForward;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends ProfiledPIDSubsystem {
    public static final double MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M = Units.inchesToMeters(48);
    public static final double ADDITIONAL_LENGTH_M = Units.inchesToMeters(30.71);

    public static final double MAX_EXTENSION_M = 0.902;
    public static final double MIN_EXTENSION_M = 0;
    public final WPI_TalonFX motor;
    public final static double SPOOL_CIRCUMFERENCE_M = Math.PI * Units.inchesToMeters(0.75); // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 4.3; // this number gives accurate values formerly 12:1
    private DoubleSupplier angleDealer_DEG;

    public static final double MAX_VELOCITY_EXTENSION = 1.5;
    public static final double MAX_ACCEL_EXTENSION = 3;
   
    public static final double kS = 0.14072;
    public static final double kV = 7.8821;
    public static final double kA = 0.45821;
    public static final double kG = 0.18613;

    // TODO: tune
    public static final double kP = 2.2;
    public static final double kI = 0.1;
    public static final double kD = 0;

    public enum ElevatorMode {
        VELOCITY , POSITION, DISABLED
    }

    private ElevatorMode extensionMode;
    private double metersPerSecond;


    private final ArmFeedforward extensionfeedforward =
        new ArmFeedforward(
            kS, kG, kV, kA
            );
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtensionSubsystem(WPI_TalonFX motor, DoubleSupplier angleDegreesDealer) {
        super(
            new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION) //TODO: tune
            )
        );
        this.motor = motor;
        metersPerSecond = 0;
        this.angleDealer_DEG = angleDegreesDealer;
        extensionMode = ElevatorMode.DISABLED;


        setGoal(MIN_EXTENSION_M);
    }

    public void setVelocity(double metersPerSecond) {
        this.extensionMode = ElevatorMode.VELOCITY;
        this.metersPerSecond = metersPerSecond;
    }

    public void setArmExtension(double meters) {
        this.extensionMode = extensionMode.POSITION;
        setGoal(meters);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      double feedforward = extensionfeedforward.calculate(Units.degreesToRadians(this.angleDealer_DEG.getAsDouble()), setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      motor.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
      return getCurrentExtensionLength();
    }

    @Override
    public void periodic() {
        switch(extensionMode) {
            case DISABLED:
                motor.stopMotor();
                break;
            case POSITION:
                useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
                break;
            case VELOCITY:
                motor.setVoltage(extensionfeedforward.calculate(Units.degreesToRadians(this.angleDealer_DEG.getAsDouble()),
                metersPerSecond));
                break;
        }
    }
    
    /**
     * Returns the motor controller used to extend the arm.
     *
     * @return the motor controller used to extend the arm
     */
    public WPI_TalonFX getMotor() {
        return this.motor;
    }

    public void initializeEncoderPositions(double meters) {
        this.motor.setSelectedSensorPosition(extensionLengthToRevs(meters) * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);
    }

    public double getCurrentExtensionLength() {
        return revsToExtensionLength(this.motor.getSelectedSensorPosition() / RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);
    }

    public void setVoltageSafely(double voltage) {
        if ((java.lang.Math.cos(Units.degreesToRadians(angleDealer_DEG.getAsDouble())) * (getCurrentExtensionLength()) + ADDITIONAL_LENGTH_M) > MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M  && voltage > 0) {
            System.err.println("WE DO NOT LIKE GAMING");
            this.motor.setVoltage(0);
            return;
        };

        this.motor.setVoltage(voltage);
    }

    public double revsToExtensionLength(double rotations) {
        final double number_of_spool_rotations = rotations/GEARBOX_RATIO;
        final double extensionLength_M = number_of_spool_rotations * SPOOL_CIRCUMFERENCE_M;
        return extensionLength_M;
    }

    public double extensionLengthToRevs(double extension_meters) {
        return extension_meters / SPOOL_CIRCUMFERENCE_M * GEARBOX_RATIO;
    }

    public boolean isArmAtExtension(double extensionLengthMeters) {
        if (getCurrentExtensionLength() >= extensionLengthMeters - .1 && getCurrentExtensionLength() <= extensionLengthMeters + .1) {
            return true;
        } else {
            return false;
        }
    }
}

