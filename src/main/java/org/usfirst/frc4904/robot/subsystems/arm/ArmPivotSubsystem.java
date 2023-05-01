package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends ProfiledPIDSubsystem {
    public static final double HARD_STOP_ARM_ANGLE = -38;
    public static final double HARD_STOP_BACK = (HARD_STOP_ARM_ANGLE* -1) + 180;

    // constants for small sprocket
    // public static final double GEARBOX_RATIO = 48; //48:1, 48 rotations of motor = 360 degrees
    // public static final double GEARBOX_SLACK_DEGREES = 6;
    // public static final double MAX_EXTENSION_M = Units.inchesToMeters(39.5);
    // public static final double MIN_EXTENSION_M = 0;

    // public static final double kS = 0;
    // public static final double kV = 0.86;
    // public static final double kA = 0.01;
    
    // public static final double kG_retracted = 0.43;
    // public static final double kG_extended = 1.08;

    // public static final double kP = 0.04;
    // public static final double kI = 0.01;
    // public static final double kD = 0;




    // constants for big sprocket, assuming it's 4x the little sprocket
    public static final double GEARBOX_RATIO = 48 * 60/26; // big sprocket
    public static final double GEARBOX_SLACK_DEGREES = 6;    // todo

    public static final double kS = 0.10126;
    // public static final double kS_extended = .20586;

    public static final double MAX_VELOCITY_PIVOT = 16;
    public static final double MAX_ACCEL_PIVOT = 10;

    
    public static final double kV = 1.8894;
    // public static final double kV_extended = 1.7361;


    public static final double kA = 0.048547; //extended: .12082
    // public static final double kA_extended = .12082;
    
    public static final double kG_retracted = 0.32;
    public static final double kG_extended = 0.6;

    // TODO: tune
    public static final double kP = 0.06;//;//0.04; //extended: .36915 retracted: .01464
    public static final double kI = 0.02;//0.01;
    public static final double kD = 0;

    public enum ArmMode {
        VELOCITY , POSITION, DISABLED
    }

    private ArmMode armMode;
    private double radiansPerSecond;

    public final DoubleSupplier extensionDealerMeters;
    public final WPI_TalonFX left;
    public final WPI_TalonFX right;


    private final TelescopingArmPivotFeedForward pivotFeedForward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);

    public ArmPivotSubsystem(WPI_TalonFX left, WPI_TalonFX right) {
        super(new ProfiledPIDController(kP, kI, kD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT)));
        this.left = left;
        this.right = right;
        this.extensionDealerMeters = () -> RobotMap.Component.armExtension.getCurrentExtensionLength();
        armMode = ArmMode.DISABLED;

        setGoal(HARD_STOP_ARM_ANGLE);

    }
    public double getAverageTicks() {
        //TODO: check inversion (average ticks might be marginally more accurate)
        return (left.getSelectedSensorPosition()/2 + right.getSelectedSensorPosition()/2);
    }
    public double getCurrentAngleDegrees() {
        // return slackyEncoder.getRealPosition();
        return (motorRevsToAngle(getAverageTicks()/RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV) * .932 - 1.66);

    }

    /**
     * Expects sensors to be zeroed at forward hard-stop.
     */
    public void initializeEncoderPositions() {
        left.setSelectedSensorPosition(angleToMotorRevs(HARD_STOP_ARM_ANGLE) * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);
        right.setSelectedSensorPosition(angleToMotorRevs(HARD_STOP_ARM_ANGLE) * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);
    }

    public static double motorRevsToAngle(double revs) {
        final double degrees_per_rotation = 360/GEARBOX_RATIO;
        final double degrees = revs * degrees_per_rotation;
        return degrees;
    }

    public static double angleToMotorRevs(double angle) {
        return angle / (360/GEARBOX_RATIO);
    }

    public boolean isArmAtRotation(double armRotationDegrees) {
        if (getCurrentAngleDegrees() >= armRotationDegrees - 2 && getCurrentAngleDegrees() <= armRotationDegrees + 2) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      double feedforward = pivotFeedForward.calculate(extensionDealerMeters.getAsDouble()/ArmExtensionSubsystem.MAX_EXTENSION_M, Units.degreesToRadians(getCurrentAngleDegrees()), setpoint.velocity, 0);
      // Add the feedforward to the PID output to get the motor output
      left.setVoltage(output + feedforward);
      right.setVoltage(output + feedforward);
    }

    @Override
    public double getMeasurement() {
        return Units.degreesToRadians(getCurrentAngleDegrees());
    }

    public WPI_TalonFX getLeftMotor() {
        return this.left;
    }

    public void setVelocity(Double radiansPerSecond) {
        this.armMode = ArmMode.VELOCITY;
        this.radiansPerSecond = radiansPerSecond;
    }

    public void setArmAngleRadians(double radians) {
        this.armMode = ArmMode.POSITION;
        setGoal(radians);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch(armMode) {
            case POSITION:
                useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
                break;
            case VELOCITY:
                left.setVoltage(pivotFeedForward.calculate(extensionDealerMeters.getAsDouble()/ArmExtensionSubsystem.MAX_EXTENSION_M, Units.degreesToRadians(getCurrentAngleDegrees()), this.radiansPerSecond, 0));
                right.setVoltage(pivotFeedForward.calculate(extensionDealerMeters.getAsDouble()/ArmExtensionSubsystem.MAX_EXTENSION_M, Units.degreesToRadians(getCurrentAngleDegrees()), this.radiansPerSecond, 0));

                
                break;
            case DISABLED:
                left.stopMotor();
                right.stopMotor();
                break;
        };
    }
}
