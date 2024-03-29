package org.usfirst.frc4904.robot.seenoevil;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.usfirst.frc4904.robot.seenoevil.Constants.DriveConstants;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Balance extends CommandBase {
    private static float RAMP_START_ANGLE = 9.0f;
    private static float RAMP_BALANCE_TOLERANCE = 4.0f;

    private final SimpleMotorFeedforward feedforward;
    private final PIDController closedLeft;
    private final PIDController closedRight;
    private final TrapezoidProfile profile;
    private final AHRS gyro;

    private final Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds;
    private final BiConsumer<Double, Double> outputVolts;

    static double initial_timestamp_system = 0;
    static double prevtime = 0;
    static DifferentialDriveWheelSpeeds prevSpeed = new DifferentialDriveWheelSpeeds();
    static DifferentialDriveWheelSpeeds speed = new DifferentialDriveWheelSpeeds();

    private double initial_timestamp;
    private boolean onRamp = false;
    private boolean balanced = false;
    private double max_velocity;
    private static double s_setpoint;

    private boolean logging = true;
    public Balance(AHRS gyro, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts, double maxAccel, double maxVelocity, Subsystem... requirements) {
        addRequirements(requirements);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAccel, 0), new TrapezoidProfile.State(maxVelocity, maxAccel), new TrapezoidProfile.State(0, 0));
        this.s_setpoint = max_velocity;
        this.feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
        this.closedLeft = new PIDController(DriveConstants.kPDriveVel, 0.01, 0);
        this.closedRight = new PIDController(DriveConstants.kPDriveVel, 0.01, 0);
        this.gyro = gyro;
        this.max_velocity = maxVelocity;

        this.wheelSpeeds = wheelSpeeds;
        this.outputVolts = outputVolts;
    }

    @Override
    public void initialize() {
        initial_timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsed_time = (Timer.getFPGATimestamp() - initial_timestamp);
        double setpoint = max_velocity;
        // double setpoint = profile.calculate(elapsed_time).position;
        DifferentialDriveWheelSpeeds currentWheelspeeds = wheelSpeeds.get();

        double leftOutput  = closedLeft .calculate(currentWheelspeeds.leftMetersPerSecond, setpoint)
                            + feedforward.calculate(setpoint);
        double rightOutput = closedRight.calculate(currentWheelspeeds.rightMetersPerSecond, setpoint)
                            + feedforward.calculate(setpoint);
    
        outputVolts.accept(leftOutput, rightOutput);

        if (Math.abs(gyro.getPitch()) > RAMP_START_ANGLE && !onRamp) {
            onRamp = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (onRamp && Math.abs(gyro.getPitch()) < RAMP_BALANCE_TOLERANCE) {
            balanced = true;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        outputVolts.accept(0.0, 0.0);
    }
}
