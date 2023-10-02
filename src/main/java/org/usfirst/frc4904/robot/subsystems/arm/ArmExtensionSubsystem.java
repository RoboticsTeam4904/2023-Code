// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.commands.TriggerCommandFactory;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import static org.usfirst.frc4904.robot.Utils.nameCommand;

public class ArmExtensionSubsystem extends SubsystemBase {
    public static final double MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M = Units.inchesToMeters(48);
    public static final double ADDITIONAL_LENGTH_M = Units.inchesToMeters(30.71);

    public static final double MAX_EXTENSION_M = 0.902;
    public static final double MIN_EXTENSION_M = 0;
    public final TalonMotorSubsystem motor;
    public final static double SPOOL_CIRCUMFERENCE_M = Math.PI * Units.inchesToMeters(0.75); // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 4.3; // this number gives accurate values formerly 12:1
    private final ArmFeedforward feedforward;
    private DoubleSupplier angleDealer_DEG;
   
    public static final double kS = 0.14072;
    public static final double kV = 7.8821;
    public static final double kA = 0.45821;
    public static final double kG = 0.18613;

    // TODO: tune
    public static final double kP = 2.2;
    public static final double kI = 0.1;
    public static final double kD = 0;
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtensionSubsystem(TalonMotorSubsystem motor, DoubleSupplier angleDegreesDealer) {
        this.motor = motor;
        this.feedforward = new ArmFeedforward(kS, kG, kV);
        this.angleDealer_DEG = angleDegreesDealer;
    }

    public void initializeEncoderPositions(double meters) {
        this.motor.zeroSensors(extensionLengthToRevs(meters));
    }

    public double getCurrentExtensionLength() {
        return revsToExtensionLength(motor.getSensorPositionRotations());
    }

    public void setVoltageSafely(double voltage) {
        // if ((java.lang.Math.cos(Units.degreesToRadians(angleDealer_DEG.getAsDouble())) * (getCurrentExtensionLength()) + ADDITIONAL_LENGTH_M) > MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M  && voltage > 0) { //TODO: Tune
        //     System.err.println("WE DO NOT LIKE GAMING");
        //     this.motor.setVoltage(0);
        //     return;
        // };

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

    public Command c_controlVelocity(DoubleSupplier metersPerSecondSupplier) {
        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(
                Units.degreesToRadians(this.angleDealer_DEG.getAsDouble()),
                metersPerSecondSupplier.getAsDouble()
            );
            SmartDashboard.putNumber("arm extension ff", ff);
            SmartDashboard.putNumber("extension velocity", revsToExtensionLength(motor.getSensorVelocityRPM()));
            setVoltageSafely(ff);
        });
        cmd.setName("arm - c_controlVelocity");
        cmd.addRequirements(motor);
        return cmd;
    }

    public Command c_holdExtension(double extensionLengthMeters, double maxVelocity, double maxAcceleration, Supplier<Command> onArrivalCommandDealer) {
        ezControl controller = new ezControl(
            kP, kI, kD, 
            (double position, double velocity) -> this.feedforward.calculate(
                Units.degreesToRadians(angleDealer_DEG.getAsDouble()) - (Math.PI/2),
                velocity,
                0
            )) {
                @Override
                public void updateSetpoint(double setpoint, double setpoint_dt) {
                    super.updateSetpoint(setpoint * 0.968 - 0.0853, setpoint_dt);
            }
        };
        
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), 
                                                        new TrapezoidProfile.State((extensionLengthMeters - 0.0853)/0.968, 0), 
                                                        new TrapezoidProfile.State((getCurrentExtensionLength() - 0.0853)/0.968, 0));
        var cmd = new ezMotion(controller, this::getCurrentExtensionLength, motor::setVoltage, () -> (double t) -> new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this, motor);
        cmd.setName("arm - c_holdExtension");
        return onArrivalCommandDealer == null ? cmd : nameCommand("extension w/ onArrival", new ParallelCommandGroup(
            cmd,
            new SequentialCommandGroup(
                new WaitCommand(profile.totalTime()),
                new TriggerCommandFactory("arm extension", onArrivalCommandDealer)
            ))
        );
    }
}

