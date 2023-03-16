package org.usfirst.frc4904.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.SmartMotorSubsystem;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Chassis2023 extends WestCoastDrive {
    public static final double RAMP_START_ANGLE = 9.0;
    public static final double RAMP_BALANCE_TOLERANCE = 4.0;
    public Chassis2023(double trackWidthMeters, double motorToWheelGearRatio, double wheelDiameterMeters,
    double drive_kP, double drive_kI, double drive_kD,
    AHRS navx, TalonMotorSubsystem leftMotorSubsystem, TalonMotorSubsystem rightMotorSubsystem) {
        super(trackWidthMeters, motorToWheelGearRatio, wheelDiameterMeters, drive_kP, drive_kI, drive_kD, navx, leftMotorSubsystem, rightMotorSubsystem);
    }
    // public Command c_followSpline(Trajectory trajectory, double ffks, double ffkv, double ffka,
    // double ramsete_b, double ramsete_zeta){
    //     RamseteCommand ramseteCommand = new RamseteCommand(
	// 	// RamseteCommand ramseteCommand = new RamseteCommand(
	// 		trajectory,
	// 		()->this.getPoseMeters(),
	// 		new RamseteController(ramsete_b, ramsete_zeta),
	// 		new SimpleMotorFeedforward(
	// 			ffks,
	// 			ffkv,
	// 			ffka),
	// 		this.kinematics,
	// 		()->this.getWheelSpeeds(),
	// 		new PIDController(0, 0, 0),
	// 		new PIDController(0, 0, 0),
	// 		// RamseteCommand passes volts to the callback
    //         (leftV, rightV) -> this.setWheelVoltages(leftV, rightV),
	// 		// (leftV, rightV) -> { this.leftMotors.leadMotor.setVoltage(leftV); this.rightMotors.leadMotor.setVoltage(rightV); },
	// 		this,
    //         this.leftMotors,
    //         this.rightMotors
    //         );
	
	// 	// Reset odometry to the starting pose of the trajectory.
	// 	Pose2d initialPose = trajectory.getInitialPose();
	// 	this.resetPoseMeters(initialPose);
	// 	SmartDashboard.putString("initial pose", initialPose.toString());
	// 	// return new Gaming(m_robotDrive);
	// 	// Run path following command, then stop at the end.
	// 	// return Commands.run(() -> m_robotDrive.tankDriveVolts(1, 1), m_robotDrive);
	// 	//return Commands.runOnce(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive);
	// 	//return Commands.runOnce(() -> Component.testTalon.setVoltage(6));
	// 	return ramseteCommand;
	// 		//.andThen(() -> this.setWheelVoltages(0, 0));
    // }
	
    public Command c_driveUntilBalanced(AHRS gyro, double volts){
            return this.c_controlWheelVoltages(()->new DifferentialDriveWheelVoltages(volts,volts))
                .until(()-> Math.abs(gyro.getPitch()) > RAMP_START_ANGLE)
                .andThen(c_controlWheelVoltages(()->new DifferentialDriveWheelVoltages(volts,volts)))
                    .until(()-> Math.abs(gyro.getPitch()) < RAMP_BALANCE_TOLERANCE)
                .andThen(()->this.setWheelVoltages(0,0));
        }
    
    
}
