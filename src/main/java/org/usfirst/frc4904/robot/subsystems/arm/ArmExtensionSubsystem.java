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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtensionSubsystem extends SubsystemBase {
    public static final double MAXIMUM_HORIZONTAL_SAFE_EXTENSION_M = Units.inchesToMeters(48);
    public static final double ADDITIONAL_LENGTH_M = Units.inchesToMeters(30.71);

    public static final double MAX_EXTENSION_M = Units.inchesToMeters(39.5);
    public static final double MIN_EXTENSION_M = 0;
    private final WPI_TalonFX motor;
    private final static double SPOOL_DIAMETER_M = Units.inchesToMeters(0.75);
    public final static double SPOOL_CIRCUMFERENCE_M = Math.PI * SPOOL_DIAMETER_M; // Math.PI * SPOOL_DIAMETER
    private final static double GEARBOX_RATIO = 12; // 12:1 
    private final TelescopingArmExtensionFeedForward feedforward;
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
    public ArmExtensionSubsystem(WPI_TalonFX motor, DoubleSupplier angleDegreesDealer) {
        this.motor = motor;
        this.feedforward = new TelescopingArmExtensionFeedForward(kS, kG, kV, kA);
        this.angleDealer_DEG = angleDegreesDealer;
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

    public Command c_controlVelocity(DoubleSupplier metersPerSecondSupplier) {
        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(
                Units.degreesToRadians(this.angleDealer_DEG.getAsDouble()),
                metersPerSecondSupplier.getAsDouble()
            );
            setVoltageSafely(ff);
        });
        cmd.setName("arm - c_controlVelocity");
        cmd.addRequirements(this);
        return cmd;
    }

    
    public Command c_holdExtension(double extensionLengthMeters) {
        var cmd = this.run(() -> {
            double lastSpeed = 0;
            double lastTime = Timer.getFPGATimestamp();

            ProfiledPIDController controller = new ProfiledPIDController(
                kP, kI, kD,
                new TrapezoidProfile.Constraints(5, 10)); //TODO: tune
            controller.setTolerance(0.01);

            double pidVal = controller.calculate(getCurrentExtensionLength(), extensionLengthMeters);
            double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
            motor.setVoltage(
                pidVal
                + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
            lastSpeed = controller.getSetpoint().velocity;
            lastTime = Timer.getFPGATimestamp();
       });   

       return cmd;
    }
}

