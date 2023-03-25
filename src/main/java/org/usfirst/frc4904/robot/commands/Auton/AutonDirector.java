package org.usfirst.frc4904.robot.commands.Auton;


import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonDirector { //replaces seenoevil, hopefully
    public Map trajectories = trajectoryConfigs.trajectories;
    public WestCoastDrive m_robotDrive = RobotMap.Component.chassis;

    public AutonDirector(){
        
    }

    public Command runSpline(Trajectory trajectory) {
		// RamseteCommandDebug ramseteCommand = new RamseteCommandDebug(
                RamseteController EEEE = new RamseteController(RobotMap.PID.Drive.ramsete_b, RobotMap.PID.Drive.ramsete_zeta);
                EEEE.setEnabled(false);
		RamseteCommand ramseteCommand = new RamseteCommand(
			trajectory,
			m_robotDrive::getPoseMeters,
			EEEE,
			new SimpleMotorFeedforward(
				RobotMap.PID.Drive.kS,
				RobotMap.PID.Drive.kV,
				RobotMap.PID.Drive.kP),
            RobotMap.PID.Drive.kDriveKinematics,
			m_robotDrive::getWheelSpeeds,
			new PIDController(RobotMap.PID.Drive.kP, 0, 0),
			new PIDController(RobotMap.PID.Drive.kP, 0, 0),
			// RamseteCommand passes volts to the callback
			m_robotDrive::setWheelVoltages,
			m_robotDrive);
	
		// Reset odometry to the starting pose of the trajectory.
		// Pose2d initialPose = trajectory.getInitialPose();
		// SmartDashboard.putString("initial pose", initialPose.toString());
		// return new Gaming(m_robotDrive);
		// Run path following command, then stop at the end.
		// return Commands.run(() -> m_robotDrive.tankDriveVolts(1, 1), m_robotDrive);
		//return Commands.runOnce(() -> m_robotDrive.arcadeDrive(0.5, 0), m_robotDrive);
		//return Commands.runOnce(() -> Component.testTalon.setVoltage(6));
		return Commands.runOnce(() -> 		m_robotDrive.resetPoseMeters(trajectory.getInitialPose())
                                ) .andThen(                ramseteCommand)
			.andThen(() -> m_robotDrive.setWheelVoltages(0, 0));
	}

    public Trajectory getTrajectory(String trajectoryName) {
        return (Trajectory) trajectories.get(trajectoryName);
    }



	//AUTONS BELOW

	public Command balanceAuton(Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts){
        var command = new SequentialCommandGroup(     
                //1. Position arm to place gamepiece
                // TODO: options: either place the game picee, or try to flip over, shoot, and then come back so that we are in the same state

                // implement going over and shooting a cone?

            new ParallelCommandGroup(
                //3. Retract arm
                // RobotMap.Component.arm.c_posReturnToHomeDown(false),
                new SequentialCommandGroup(
                    new WaitCommand(1), //TODO: set wait time to allow arm to get started before moving?
                    //4. Drive forward past ramp
                    runSpline(getTrajectory("past_ramp")),

                    //5. Drive back to get partially on ramp
                    runSpline(getTrajectory("back_to_ramp"))
                )
            )
        //     new Balance(RobotMap.Component.navx, wheelSpeeds, outputVolts, 1, -0.1)
            //6. balance code here
        );
        
        return command;
        }

        public Command balanceAutonAndShootCube(Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, BiConsumer<Double, Double> outputVolts){
            
                var command = new SequentialCommandGroup(     
                        //1. Position arm to place gamepiece
                        // TODO: options: either place the game picee, or try to flip over, shoot, and then come back so that we are in the same state
        
                        // implement going over and shooting a cone?
        
                    new ParallelCommandGroup(
                        //3. Retract arm
                        // RobotMap.Component.arm.c_posReturnToHomeDown(false),
                        RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(180-15, 150, 200).getFirst(),
                        new SequentialCommandGroup(
                            new WaitCommand(RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(180-15, 150, 200).getSecond()),
                            RobotMap.Component.intake.c_holdVoltage(4.5).withTimeout(0.5),
                            RobotMap.Component.arm.armPivotSubsystem.c_holdRotation(0, 150, 150).getFirst().withTimeout(0.8)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(2.5), //TODO: set wait time to allow arm to get started before moving?
                            //4. Drive forward past ramp
                            runSpline(getTrajectory("past_ramp")),
        
                            //5. Drive back to get partially on ramp
                            runSpline(getTrajectory("back_to_ramp"))
                        )
                    )
                //     new Balance(RobotMap.Component.navx, wheelSpeeds, outputVolts, 1, -0.1)
                    //6. balance code here
                );
                
                return command;
                }

        public Command notBalanceAuton(){
            var command = new SequentialCommandGroup(     
                    //1. Position arm to place gamepiece
                    RobotMap.Component.arm.placeCube(2, true) //TODO: set actual timeout
                    ,
                new ParallelCommandGroup(
                    //3. Retract arm
                    RobotMap.Component.arm.c_posReturnToHomeUp(false),
                    new SequentialCommandGroup(
                        new WaitCommand(1), //TODO: set wait time to allow arm to get started before moving?
                        //4. Drive forward past ramp
                        runSpline(getTrajectory("past_ramp"))
                    )
                )
            );
            
            return command;
            }

}
