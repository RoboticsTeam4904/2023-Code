// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
    public static final double MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M = Units.inchesToMeters(48);
    public static final double ADDITIONAL_LENGTH_M = Units.inchesToMeters(30.71);

    public static final double MAX_EXTENSION_M = Units.inchesToMeters(39.5);
    public static final double MIN_EXTENSION_M = 0;
    private final TalonMotorSubsystem motor;
    private final static double SPOOL_DIAMETER_M = Units.inchesToMeters(0.75);
    public final static double SPOOL_CIRCUMFERENCE_M = Math.PI * SPOOL_DIAMETER_M; // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 12; // 12:1 
    private final ArmFeedforward feedforward;
    private DoubleSupplier angleDealer_DEG;
   
    // TODO: recharacterize -- current values may be incorrect
    public static final double kS = 0.21679;
    public static final double kV = 8.2054;
    public static final double kA = 0.17697;
    public static final double kG = 0.26169;

    // TODO: tune
    public static final double kP = 0.01;
    public static final double kI = 0.001;
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
    
    /**
     * Returns the motor controller used to extend the arm.
     *
     * @return the motor controller used to extend the arm
     */
    public TalonMotorSubsystem getMotor() {
        return motor;
    }

    public void initializeEncoderPositions(double meters) {
        this.motor.zeroSensors(extensionLengthToRevs(meters));
    }

    public double getCurrentExtensionLength() {
        return revsToExtensionLength(motor.getSensorPositionRotations());
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

    public Command c_controlVelocity(DoubleSupplier metersPerSecondSupplier) {
        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(
                Units.degreesToRadians(this.angleDealer_DEG.getAsDouble()),
                metersPerSecondSupplier.getAsDouble()
            );
            // SmartDashboard.putNumber("arm extension ff", ff);
            // SmartDashboard.putNumber("extension velocity", revsToExtensionLength(motor.getSensorVelocityRPM()));
            setVoltageSafely(ff);
        });
        cmd.setName("arm - c_controlVelocity");
        cmd.addRequirements(motor);
        return cmd;
    }

    public Pair<Command, Double> c_holdExtension(double extensionLengthMeters, double maxVelocity, double maxAcceleration) {
        ezControl controller = new ezControl(kP, kI, kD, 
                                            (double position, double velocity) -> this.feedforward.calculate(
                                                Units.degreesToRadians(angleDealer_DEG.getAsDouble()) - Math.PI/2,
                                                velocity,
                                                0
                                            ));
        
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), 
                                                        new TrapezoidProfile.State(extensionLengthMeters, 0), 
                                                        new TrapezoidProfile.State(getCurrentExtensionLength(), 0));
        var cmd = new ezMotion(controller, this::getCurrentExtensionLength, motor::setVoltage, (double t) -> new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this, motor);
        cmd.setName("arm - c_holdExtension");
        return new Pair<Command, Double>(cmd, profile.totalTime());
    }
}

