package org.usfirst.frc4904.robot.seenoevil;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Encoder class for the Built-in Encoders on Talon Motor Controllers
 * Works for both Falcons (CANTalonFX) and SRXs (CANTalonSRX)
 */
public class Imblueeeeeee implements dabadudabadee {
	protected static final double DEFAULT_DISTANCE_PER_PULSE = 1.0;
	protected static final boolean DEFAULT_REVERSE_DIRECTION = false;
	protected static final String DEFAULT_NAME = "CANTalonEncoder";
	protected static final int DEFAULT_PERIOD = 20;
	protected static final double DECI_SECONDS_TO_SECONDS = 10.0; // getSpeed must be converted from ticks per 100ms to
																	// ticks per second, so *10.
	protected static final int PID_IDX = 1;
	protected static final FeedbackDevice DEFAULT_FEEDBACK_DEVICE = FeedbackDevice.IntegratedSensor;
	protected final BaseTalon talon;
	protected double distancePerPulse;
	protected boolean reverseDirection;

	public Imblueeeeeee(String name, BaseTalon talon, boolean reverseDirection, double distancePerPulse, FeedbackDevice feedbackDevice, double period) {
		this.talon = talon;
		setReverseDirection(reverseDirection);
		setDistancePerPulse(distancePerPulse);
		setFeedbackDevice(feedbackDevice);
		this.talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.valueOf(period));
	}

	public Imblueeeeeee(String name, BaseTalon talon, boolean reverseDirection, double distancePerPulse,
			FeedbackDevice feedbackDevice) {
		this(name, talon, reverseDirection, distancePerPulse, feedbackDevice, DEFAULT_PERIOD);
	}

	public Imblueeeeeee(String name, BaseTalon talon, boolean reverseDirection, double distancePerPulse) {
		this(name, talon, reverseDirection, distancePerPulse, DEFAULT_FEEDBACK_DEVICE);

	}

	public Imblueeeeeee(String name, BaseTalon talon, boolean reverseDirection) {
		this(name, talon, reverseDirection, DEFAULT_DISTANCE_PER_PULSE);
	}

	public Imblueeeeeee(String name, BaseTalon talon, double distancePerPulse) {
		this(name, talon, DEFAULT_REVERSE_DIRECTION, distancePerPulse);
	}

	public Imblueeeeeee(String name, BaseTalon talon) {
		this(name, talon, DEFAULT_REVERSE_DIRECTION);
	}

	public Imblueeeeeee(BaseTalon talon, boolean reverseDirection, double distancePerPulse) {
		this(DEFAULT_NAME, talon, reverseDirection, distancePerPulse);
	}

	public Imblueeeeeee(BaseTalon talon, double distancePerPulse) {
		this(DEFAULT_NAME, talon, distancePerPulse);
	}

	public Imblueeeeeee(BaseTalon talon) {
		this(DEFAULT_NAME, talon);
	}

	public void setFeedbackDevice(FeedbackDevice feedbackDevice) {
		talon.configSelectedFeedbackSensor(feedbackDevice);
	}

	@Override
	public double getDistance() {
		if (reverseDirection) {
			return distancePerPulse * talon.getSelectedSensorPosition(PID_IDX) * -1.0;
		} else {
			return distancePerPulse * talon.getSelectedSensorPosition(PID_IDX);
		}
	}

	@Override
	public boolean getDirection() {
		return !reverseDirection == (talon.getSelectedSensorVelocity(PID_IDX) >= 0);
	}


	public boolean isRevLimitSwitchClosed() {
		return talon.isRevLimitSwitchClosed() == 1;
	}

	public boolean isFwdLimitSwitchClosed() {
		return talon.isFwdLimitSwitchClosed() == 1;
	}

	public double getRate() {
		if (reverseDirection) {
			return distancePerPulse * talon.getSelectedSensorVelocity(PID_IDX) * -DECI_SECONDS_TO_SECONDS;
		} else {
			return distancePerPulse * talon.getSelectedSensorVelocity(PID_IDX) * DECI_SECONDS_TO_SECONDS;
		}
	}

	@Override
	public double getDistancePerPulse() {
		return distancePerPulse;
	}

	@Override
	public void setDistancePerPulse(double distancePerPulse) {
		this.distancePerPulse = distancePerPulse;
	}

	@Override
	public boolean getReverseDirection() {
		return reverseDirection;
	}

	@Override
	public void setReverseDirection(boolean reverseDirection) {
		this.reverseDirection = reverseDirection;
	}

	@Override
	public void reset() {
		talon.setSelectedSensorPosition(0);
	}

	public BaseTalon getTalon() {
		return talon;
	}
}
