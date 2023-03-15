package org.usfirst.frc4904.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc4904.robot.FunnyNumber.funnynumber;;
public class ArmPivotSubsystem extends SubsystemBase {
    public static final double INITIAL_ARM_ANGLE = -37.25;
    public static final double GEARBOX_RATIO = 48; //48:1, 48 rotations of motor = 360 degrees
    public static final double MAX_EXTENSION = 39.5;

    public static final double MAX_ANGULAR_ACCEL = funnynumber("Pivot: max a", Math.PI/5);   // rad/sec      TUESDAY: consevative, how fast should we go?
    public static final double MAX_ANGULAR_VEL = funnynumber("Pivot: max v", Math.PI/5);     // rad/sec^2    TUESDAY: this is conservative; how fast should we go?

    // recalc values
    // public static final double kS = funnynumber("Pivot: ks", 0);
    // public static final double kV = funnynumber("Pivot: kv", 0.86);
    // public static final double kA = funnynumber("Pivot: kA", 0.01);
    // public static final double kG_retracted = funnynumber("Pivot: kG retracted", 0.46);
    // public static final double kG_extended  = funnynumber("Pivot: kG extended",  0.86);
    // TODO: REMOVE
    public static final double kS = funnynumber("Pivot: ks", 0.05);
    public static final double kV = funnynumber("Pivot: kv", 0.016);
    public static final double kA = funnynumber("Pivot: kA", 0.0006);
    public static final double kG_retracted = funnynumber("Pivot: kG retracted", 0);
    public static final double kG_extended  = funnynumber("Pivot: kG extended",  0);

    // TUESDAY TODO: tune
    public static final double kP = 0.002;
    public static final double kI = 0;
    public static final double kD = 0;

    public final TalonMotorSubsystem armMotorGroup;
    public final TelescopingArmPivotFeedForward feedforward;
    public final DoubleSupplier extensionDealer;
    public ArmPivotSubsystem(TalonMotorSubsystem armMotorGroup, DoubleSupplier extensionDealer) {
        this.armMotorGroup = armMotorGroup;
        this.extensionDealer = extensionDealer;
        this.feedforward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);
    }

    public double getCurrentAngleDegrees() {
        return motorRevsToAngle(armMotorGroup.getSensorPositionRotations());
    }

    public void initEncoderPositions() {
        armMotorGroup.zeroSensors(angleToMotorRevs(INITIAL_ARM_ANGLE));
    }

    public double motorRevsToAngle(double revs) {
        final double degrees_per_rotation = 360/GEARBOX_RATIO;
        final double degrees = revs * degrees_per_rotation;
        return degrees;
    }

    public double angleToMotorRevs(double angle) {
        return angle / (360/GEARBOX_RATIO);
    }

    public Command c_feedforwardTest(DoubleSupplier velocityDealer) {
        return this.run(() -> {
            var ff = this.feedforward.calculate(
                extensionDealer.getAsDouble()/MAX_EXTENSION,
                Units.degreesToRadians(getCurrentAngleDegrees()),
                velocityDealer.getAsDouble(),
                0
            );
            SmartDashboard.putNumber("eeeeee ff output", ff);
            System.out.println("i am inside my home" + new Double(ff).toString());
            // this.armMotorGroup.setVoltage(ff);
            this.armMotorGroup.leadMotor.setVoltage(ff);
        });
    }

    // public Command c_controlRotation(DoubleSupplier degreesFromHorizontalDealer) {
    //     armMotorGroup.configPIDF(kP, kI, kD, 0, 2048, funnynumber("Pivot DMP peakOutput", 0.1), null);
    //     armMotorGroup.configDMP(0, Units.radiansPerSecondToRotationsPerMinute(MAX_ANGULAR_VEL)*GEARBOX_RATIO, Units.radiansPerSecondToRotationsPerMinute(MAX_ANGULAR_ACCEL)*GEARBOX_RATIO, kS, null);
    //     return armMotorGroup.c_controlPosition(
    //         () -> Units.degreesToRotations(degreesFromHorizontalDealer.getAsDouble())*GEARBOX_RATIO,
    //         () -> this.feedforward.calculate(
    //             extensionDealer.getAsDouble()/MAX_EXTENSION,
    //             Units.degreesToRadians(getCurrentAngleDegrees()),
    //             armmotorGroup.leadMotor.getDynam
    //             0
    //         )
    //     );
    //     // could also make native version of holdRotation using DMP, or ezMotion version of this 
    // }

    public Command c_holdRotation(double degreesFromHorizontal) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityRadPerSec) -> this.feedforward.calculate(
                extensionDealer.getAsDouble()/MAX_EXTENSION,
                getCurrentAngleDegrees() * Math.PI/180,
                velocityRadPerSec,
                0
            )
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL, MAX_ANGULAR_ACCEL),
            new TrapezoidProfile.State(Units.degreesToRadians(degreesFromHorizontal), 0),
            new TrapezoidProfile.State(Units.degreesToRadians(getCurrentAngleDegrees()), 0)
        );

        return new ezMotion(controller, () -> Units.degreesToRadians(this.getCurrentAngleDegrees()), armMotorGroup::setVoltage,
                (double t) ->  new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this);
    }
}
