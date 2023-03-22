package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
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
    public static final double MAX_EXTENSION_M = Units.inchesToMeters(39.5);
    public static final double MIN_EXTENSION_M = 0;

    public static final double kS = 0;
    public static final double kV = 1.98;
    public static final double kA = 0.03;
    
    public static final double kG_retracted = 0.32;
    public static final double kG_extended = 1.05;

    // TODO: tune
    public static final double kP = 0.04;
    public static final double kI = 0.01;
    public static final double kD = 0;





    public final MotorControllerGroup armMotorGroup;
    public final TelescopingArmPivotFeedForward feedforward;
    public final DoubleSupplier extensionDealerMeters;
    public final WPI_TalonFX encoder;
    private final EncoderWithSlack slackyEncoder;

    public ArmPivotSubsystem(MotorControllerGroup armMotorGroup, WPI_TalonFX encoder, DoubleSupplier extensionDealerMeters) {
        this.armMotorGroup = armMotorGroup;
        this.encoder = encoder;
        this.extensionDealerMeters = () -> extensionDealerMeters.getAsDouble();
        this.feedforward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);
        this.slackyEncoder = new EncoderWithSlack(
            GEARBOX_SLACK_DEGREES,
            () -> encoder.getSelectedSensorPosition(),
            Units.rotationsToDegrees(1/GEARBOX_RATIO),
            true
        );
    }

    public double getCurrentAngleDegrees() {
        // return slackyEncoder.getRealPosition();
        return motorRevsToAngle(encoder.getSelectedSensorPosition() * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);

    }

    /**
     * Expects sensors to be zeroed at forward hard-stop.
     */
    public void initializeEncoderPositions() {
        encoder.setSelectedSensorPosition(angleToMotorRevs(HARD_STOP_ARM_ANGLE) * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV);
        slackyEncoder.zeroSlackDirection(true);
    }

    public static double motorRevsToAngle(double revs) {
        final double degrees_per_rotation = 360/GEARBOX_RATIO;
        final double degrees = revs * degrees_per_rotation;
        return degrees;
    }

    public static double angleToMotorRevs(double angle) {
        return angle / (360/GEARBOX_RATIO);
    }


    public Command c_controlAngularVelocity(DoubleSupplier degPerSecDealer) {
        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(
                extensionDealerMeters.getAsDouble()/MAX_EXTENSION_M,
                Units.degreesToRadians(getCurrentAngleDegrees()),
                Units.rotationsPerMinuteToRadiansPerSecond(Units.degreesToRotations(degPerSecDealer.getAsDouble()) * 60),
                0
            );
            SmartDashboard.putNumber("feedforward", ff);

            armMotorGroup.setVoltage(ff);
        });
        cmd.addRequirements(this);
        cmd.setName("arm - c_controlAngularVelocity");
        return cmd;
    }

    public Command c_holdRotation(double degreesFromHorizontal) {
        var cmd = this.run(() -> {
            double lastSpeed = 0;
            double lastTime = Timer.getFPGATimestamp();

            ProfiledPIDController controller = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(5, 10)); //TODO: tune
            controller.setTolerance(0.01);

            double pidVal = controller.calculate(getCurrentAngleDegrees(), degreesFromHorizontal);
            double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
            armMotorGroup.setVoltage(
                pidVal
                + feedforward.calculate(extensionDealerMeters.getAsDouble()/MAX_EXTENSION_M, 
                Units.degreesToRadians(getCurrentAngleDegrees()),
                controller.getSetpoint().velocity, acceleration));
            lastSpeed = controller.getSetpoint().velocity;
            lastTime = Timer.getFPGATimestamp();
       });   

       return cmd;
    }
}
