// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.usfirst.frc4904.robot.FunnyNumber.funnynumber;

public class ArmExtensionSubsystem extends SubsystemBase {
    
    public static final double MAX_EXTENSION_ACCEL = funnynumber("Extension: max a", 0.3);  // m/s2         TUESDAY: tune
    public static final double MAX_EXTENSION_SPEED = funnynumber("Extension: max v", 0.1);  // m/s          TUESDAY: tune

    // TUESDAY: tune these
    public static final double kP = funnynumber("Extension: kP", 0.1);
    public static final double kI = funnynumber("Extension: kI", 0.0);
    public static final double kD = funnynumber("Extension: kD", 0.0);

    // TUESDAY: tune these
    // numbers from sunday sysid
    public static final double kS = funnynumber("ks", 0.21679);
    public static final double kG = funnynumber("kg", 0.26169);
    public static final double kV = funnynumber("kv", 8.2054);
    public static final double kA = funnynumber("ka", 0.17697); // TUESDAY: try using 0 for accel
    // // numbers from recalc   // TUESDAY: try using numbers from recalc
    // public static final double kS = funnynumber("ks", 0);
    // public static final double kG = funnynumber("kg", 0.26);
    // public static final double kV = funnynumber("kv", 2.83);
    // public static final double kA = funnynumber("ka", 0.02);

    public static final double SPOOL_DIAMETER = Units.inchesToMeters(1);
    public static final double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER; // Math.PI * SPOOL_DIAMETER
    public static final double GEARBOX_RATIO = 12; // 12:1 

    private final ArmFeedforward feedforward;
    private final TalonMotorSubsystem motor;
    private final DoubleSupplier angleDealer;
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtensionSubsystem(TalonMotorSubsystem motor, DoubleSupplier angleDealer) {
        this.motor = motor;
        this.feedforward = new ArmFeedforward(1, 1, -1);
        this.angleDealer = angleDealer;

    }

    public void initEncoderPositions() {
        motor.zeroSensors();
    }
    
    /**
     * Returns the motor controller used to extend the arm.
     *
     * @return the motor controller used to extend the arm
     */
    public TalonMotorSubsystem getMotor() {
        return motor;
    }

    public double getCurrentExtensionLength() {
        return revsToExtensionLength(motor.getSensorPositionRotations());
    }

    public double revsToExtensionLength(double rotations) {
        final double number_of_spool_rotations = rotations/GEARBOX_RATIO;
        final double extensionLength = number_of_spool_rotations * SPOOL_CIRCUMFERENCE;
        return extensionLength;
    }

    public Command c_holdExtension(double extensionLengthMeters) {
        ezControl controller = new ezControl(1, 1, 1, 
                                            (double position, double velocity) -> this.feedforward.calculate(angleDealer.getAsDouble() + Math.PI/2, velocity, 0));
        
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(MAX_EXTENSION_SPEED, MAX_EXTENSION_ACCEL), 
                                                        new TrapezoidProfile.State(extensionLengthMeters, 0), 
                                                        new TrapezoidProfile.State(getCurrentExtensionLength(), 0));

        return new ezMotion(controller, this::getCurrentExtensionLength, motor::setVoltage, (double t) -> new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this);
    }
}

