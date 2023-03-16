// This is the subsystem for the arm extension. 
// Code by Russell from 4904

package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat.Tuple2;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.subsystems.RequirementsSubsystemBase;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmExtensionSubsystem extends RequirementsSubsystemBase {
    
    private final TalonMotorSubsystem motor;
    private final static double SPOOL_DIAMETER = Units.inchesToMeters(0.75);
    public final static double SPOOL_CIRCUMFERENCE = Math.PI * SPOOL_DIAMETER; // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 12; // 12:1 
    private final ArmFeedforward feedforward;
    private DoubleSupplier angleDealer;
   
    // TODO: recharacterize
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    // TODO: tune
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    
    /**
     * Constructs a new ArmExtension subsystem.
     *
     * @param motor the motor controller used to extend the arm
     */
    public ArmExtensionSubsystem(TalonMotorSubsystem motor, DoubleSupplier angleDegreesDealer) {
        super(motor);
        this.motor = motor;
        this.feedforward = new ArmFeedforward(kS, kG, kV);
        this.angleDealer = angleDegreesDealer;
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

    public Command c_controlVelocity(DoubleSupplier metersPerSecondSupplier) {
        return this.run(() -> {
            var ff = this.feedforward.calculate(
                Units.degreesToRadians(this.angleDealer.getAsDouble()),
                metersPerSecondSupplier.getAsDouble()
            );
            SmartDashboard.putNumber("arm extension ff", ff);
            motor.setVoltage(ff);
        });
    }

    public Pair<Command, Double> c_holdExtension(double extensionLengthMeters, double maxVelocity, double maxAcceleration) {
        ezControl controller = new ezControl(kP, kI, kD, 
                                            (double position, double velocity) -> this.feedforward.calculate(
                                                Units.degreesToRadians(angleDealer.getAsDouble()) - Math.PI/2,
                                                velocity,
                                                0
                                            ));
        
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), 
                                                        new TrapezoidProfile.State(extensionLengthMeters, 0), 
                                                        new TrapezoidProfile.State(getCurrentExtensionLength(), 0));

        return new Pair<Command, Double>(new ezMotion(controller, this::getCurrentExtensionLength, motor::setVoltage, (double t) -> new Tuple2<Double>(profile.calculate(t).position, profile.calculate(t).velocity), this, motor), profile.totalTime());
    }
}

