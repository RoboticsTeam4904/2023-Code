package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.commands.TriggerCommandFactory;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TelescopingArmPivotFeedForward;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static org.usfirst.frc4904.robot.Utils.nameCommand;

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

    public static final double kS = 0.10126;
    // public static final double kS_extended = .20586;

    
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

    public final TalonMotorSubsystem armMotorGroup;
    public final TelescopingArmPivotFeedForward feedforward;
    public final DoubleSupplier extensionDealerMeters;

    public ArmPivotSubsystem(TalonMotorSubsystem armMotorGroup, DoubleSupplier extensionDealerMeters) {
        this.armMotorGroup = armMotorGroup;
        this.extensionDealerMeters = () -> extensionDealerMeters.getAsDouble();
        this.feedforward = new TelescopingArmPivotFeedForward(kG_retracted, kG_extended, kS, kV, kA);
    }

    public double getCurrentAngleDegrees() {
        // return slackyEncoder.getRealPosition();
        return motorRevsToAngle(armMotorGroup.getSensorPositionRotations())*0.985 - 5.18; // * 0.911 - 6.3

    }

    /**[p]
     * Expects sensors to be zeroed at forward hard-stop.
     */
    public void initializeEncoderPositions() {
        armMotorGroup.zeroSensors(angleToMotorRevs(HARD_STOP_ARM_ANGLE));
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
                extensionDealerMeters.getAsDouble()/ArmExtensionSubsystem.MAX_EXTENSION_M,
                Units.degreesToRadians(getCurrentAngleDegrees()),
                Units.rotationsPerMinuteToRadiansPerSecond(Units.degreesToRotations(degPerSecDealer.getAsDouble()) * 60),
                0
            );
            SmartDashboard.putNumber("feedforward", ff);
            for (var motor: this.armMotorGroup.followMotors) {
                motor.setVoltage(ff);
            }
            this.armMotorGroup.leadMotor.setVoltage(ff);
        });
        cmd.addRequirements(armMotorGroup);
        cmd.setName("arm - c_controlAngularVelocity");
        return cmd;
    }

    public Command c_holdRotation(double degreesFromHorizontal, double maxVelDegPerSec, double maxAccelDegPerSecSquare, Supplier<Command> onArrivalCommandDealer) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityDegPerSec) -> { 
                double brr =  this.feedforward.calculate(
                    extensionDealerMeters.getAsDouble()/ArmExtensionSubsystem.MAX_EXTENSION_M,
                    Units.degreesToRadians((getCurrentAngleDegrees() + 6.3)/0.911),
                    Units.degreesToRadians(velocityDegPerSec),
                    0
                );
                SmartDashboard.putNumber("Intended voltage", maxAccelDegPerSecSquare);
                return brr;
            }
        ){
            @Override
            public void updateSetpoint(double setpoint, double setpoint_dt) {
                super.updateSetpoint(setpoint * 0.911 - 6.3, setpoint_dt);
            }
        };

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare),
            new TrapezoidProfile.State((degreesFromHorizontal + 6.3)/0.911, 0),
            new TrapezoidProfile.State((getCurrentAngleDegrees() + 6.3)/0.911, 0)
        );

        var cmd = new ezMotion(
            controller,
            () -> this.getCurrentAngleDegrees(),
            (double volts) -> {
                // this.armMotorGroup.setVoltage(volts); // FIXME : use motorgroup setvoltage
                SmartDashboard.putNumber("Arm Volts", volts);
                this.armMotorGroup.leadMotor.setVoltage(volts);
                for (var m : this.armMotorGroup.followMotors) m.setVoltage(volts);
            },
            (double t) -> {
                SmartDashboard.putNumber("deg setpoint", profile.calculate(t).position);
                SmartDashboard.putNumber("deg velocity", profile.calculate(t).velocity);
                return new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity);
            },
        this, armMotorGroup);
        cmd.setName("arm - c_holdRotation");
        // return new Pair<Command,Double>(new ezMotion(controller, () -> this.getCurrentAngleDegrees() * Math.PI / 180, armMotorGroup::setVoltage,
        //         (double t) ->  new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this), profile.totalTime());
        return onArrivalCommandDealer == null ? cmd : nameCommand("pivot w/ onArrival: " + cmd.getName(), new ParallelCommandGroup(
            cmd,
            new SequentialCommandGroup(
                new WaitCommand(profile.totalTime()),
                new TriggerCommandFactory("arm pivot", onArrivalCommandDealer)
            ))
        );
    }
}
