package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPivotSubsystem extends SubsystemBase {
    public static final double INITIAL_ARM_ANGLE = -37.25;
    public static final double GEARBOX_RATIO = 48; //48:1, 48 rotations of motor = 360 degrees
    public static final double MAX_EXTENSION = 39.5;
    public static final double MIN_EXTENSION = 0;

    public static final double kS = 0;  // TODO
    public static final double kV = 0.86;
    public static final double kA = 0.01;
    
    public static final double kG_retracted = 0.46;
    public static final double kG_extended = 0.86;

    public static final double slack_revs = Units.degreesToRotations(3) * GEARBOX_RATIO;   // in one direction

    // TODO: tune
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public final TalonMotorSubsystem armMotorGroup;
    public final TelescopingArmPivotFeedForward feedforward;
    public final DoubleSupplier extensionDealer;
    
    private boolean wasPreviouslyGoingPositive = false; // stores the previous direction to add a "position kS" to account for slack in the arm

    public ArmPivotSubsystem(TalonMotorSubsystem armMotorGroup, DoubleSupplier extensionDealer) {
        this.armMotorGroup = armMotorGroup;
        this.extensionDealer = extensionDealer;
        this.feedforward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);
    }

    public double getCurrentAngleDegrees() {
        return motorRevsToAngle(armMotorGroup.getSensorPositionRotations());
    }

    public void zeroSensors() {
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

    public Command c_holdRotation(double degreesFromHorizontal, double maxVelDegPerSec, double maxAccelDegPerSecSquare) {
        boolean isCurrentlyGoingPositive = degreesFromHorizontal > getCurrentAngleDegrees();
        if (isCurrentlyGoingPositive != wasPreviouslyGoingPositive) {

        }

        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityRadPerSec) -> this.feedforward.calculate(
                extensionDealer.getAsDouble()/MAX_EXTENSION,
                Units.degreesToRadians(getCurrentAngleDegrees()),
                velocityRadPerSec,
                0
            )
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare),
            new TrapezoidProfile.State(degreesFromHorizontal, 0),
            new TrapezoidProfile.State(getCurrentAngleDegrees(), 0)
        );

        return new ezMotion(controller, () -> this.getCurrentAngleDegrees() * Math.PI / 180, armMotorGroup::setVoltage,
                (double t) ->  new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this);
    }
}
