package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.RobotMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderWithSlack extends SubsystemBase {   // just extends SubsystemBase so we can have a periodic
    public final DoubleSupplier encoderTicksSupplier;
    public final double slackWindow;
    public final double unitsPerRevolution;
    private double slackWindowDirection;
    private double encoderPosition;
    private double positionWithinWindow; // realPosition = encoderPosition + positionWithinWindow; positionWithinWindow = 0 when moving positively would work directly
    
    /**
     * Math class to calculate the position of a system controlled by a motor
     * through a system with static slack, eg. a high-ratio gearbox or a loose
     * chain.
     * 
     * @param slackWindowSize     The total amount that the output can move without
     *                            moving the motor shaft, in Units.
     * @param encoderRevsSupplier Function to read the position (in revolutions) of
     *                            the control motor. Will be called once per
     *                            periodic loop.
     * @param unitsPerRotation    Conversion factor from motor revolutions to Units
     *                            of the mechanism, assuming no slack.
     * @param startsAtPositiveEnd Whether the mechanism starts at the negative end
     *                            of the window (eg. no slack if we immediately move
     *                            postively). Eg, false if you push the mechanism
     *                            negatively against a brake-mode motor to reset.
     */
    public EncoderWithSlack(double slackWindowSize, DoubleSupplier encoderTicksSupplier, double unitsPerRevolution, boolean startsAtNegativeEnd) {
        this.encoderTicksSupplier = encoderTicksSupplier;
        this.slackWindow = slackWindowSize;
        this.unitsPerRevolution = unitsPerRevolution;

        this.encoderPosition = this.encoderTicksSupplier.getAsDouble() * unitsPerRevolution * RobotMap.Metrics.TALON_ENCODER_COUNTS_PER_REV;
        this.zeroSlackDirection(startsAtNegativeEnd);
    }
    /**
     * @return  The real position of the meachinsm, in Units
     */
    public double getRealPosition() {
        return encoderPosition + positionWithinWindow;
    }
    /**
     * Callback to reset the slip when gravity happens (eg. if the an arm pivot mechanism goes over vertical)
     * @param atNegativeEnd
     */
    public void zeroSlackDirection(boolean atNegativeEnd) {
        this.slackWindowDirection = atNegativeEnd ? 1 : -1;
        this.positionWithinWindow = 0;
    }

    private double clamp(double value, double bound1, double bound2) { return Math.min(Math.max(value, Math.min(bound1, bound2)), Math.max(bound1, bound2)); }
    public void periodic() {    // automatically scheduled by SubsystemBase
        var newEncoderPosition = encoderTicksSupplier.getAsDouble() * unitsPerRevolution;
        var delta = newEncoderPosition - encoderPosition;
        encoderPosition = newEncoderPosition;

        SmartDashboard.putNumber("slackencoder: delta", delta);
        SmartDashboard.putNumber("slackencoder: positionInWindow", positionWithinWindow);

        positionWithinWindow = clamp(positionWithinWindow - delta, 0, slackWindow * slackWindowDirection); // while we're within the window, the window cancels out any encoder delta
    }
}
