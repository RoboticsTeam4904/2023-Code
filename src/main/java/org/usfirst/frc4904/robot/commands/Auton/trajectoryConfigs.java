package org.usfirst.frc4904.robot.commands.Auton;

import java.util.List;
import java.util.Map;
import static java.util.Map.entry;    


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

import org.usfirst.frc4904.robot.RobotMap;

public class trajectoryConfigs {
        private static double placeholderconstant = 0; //TODO: add value for starting to move onto the ramp
        private static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                RobotMap.PID.Drive.kMaxSpeedMetersPerSecond,
                RobotMap.PID.Drive.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(RobotMap.PID.Drive.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        RobotMap.PID.Drive.kS,
                        RobotMap.PID.Drive.kV,
                        RobotMap.PID.Drive.kA),
                RobotMap.PID.Drive.kDriveKinematics,
                                10));
        private static TrajectoryConfig trajectoryConfigReversed = new TrajectoryConfig(
                RobotMap.PID.Drive.kMaxSpeedMetersPerSecond,
                RobotMap.PID.Drive.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(RobotMap.PID.Drive.kDriveKinematics)
                .addConstraint(new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                RobotMap.PID.Drive.kS,
                                RobotMap.PID.Drive.kV,
                                RobotMap.PID.Drive.kA),
                        RobotMap.PID.Drive.kDriveKinematics,
                        10))
                .setReversed(true);
        private static TrajectoryConfig trajectoryConfigSlow = new TrajectoryConfig(//TODO: add value for slow speed (and maybe accel)
                RobotMap.PID.Drive.kMaxSpeedMetersPerSecond/3,
                RobotMap.PID.Drive.kMaxAccelerationMetersPerSecondSquared/2)
                .setKinematics(RobotMap.PID.Drive.kDriveKinematics)
                .addConstraint(new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                RobotMap.PID.Drive.kS,
                                RobotMap.PID.Drive.kV,
                                RobotMap.PID.Drive.kA),
                        RobotMap.PID.Drive.kDriveKinematics,
                        10))
                .setReversed(true);
        private static TrajectoryConfig trajectoryConfigSlowReversed = new TrajectoryConfig(
                RobotMap.PID.Drive.kMaxSpeedMetersPerSecond/3,
                RobotMap.PID.Drive.kMaxAccelerationMetersPerSecondSquared/2)
                .setKinematics(RobotMap.PID.Drive.kDriveKinematics)
                .addConstraint(new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                RobotMap.PID.Drive.kS,
                                RobotMap.PID.Drive.kV,
                                RobotMap.PID.Drive.kA),
                        RobotMap.PID.Drive.kDriveKinematics,
                        10))
                .setReversed(true);
        public static Map<String, Trajectory> trajectories = Map.ofEntries(
                entry("sickle", TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        // List.of(new Translation2d(0.33*dist, .15*dist), new Translation2d(0.66*dist, -.15*dist)),
                        List.of(new Translation2d(1, -1), new Translation2d(2, -1)),

                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(2, 0, new Rotation2d(Math.PI/2)),
                        trajectoryConfig)
                ),
                entry("straight_forward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(new Translation2d(1, 0)),
                        new Pose2d(2, 0, new Rotation2d(0)),
                        trajectoryConfig
                )),
                entry("straight_backward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Math.PI)),
                        List.of(new Translation2d(1, 0)),
                        new Pose2d(2, 0, new Rotation2d(Math.PI)),
                        trajectoryConfigReversed
                )),
                entry("turn_right", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(1, -1, new Rotation2d(-Math.PI/2)),
                        trajectoryConfig
                )),
                entry("past_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(4, 0, new Rotation2d(0)),
                        trajectoryConfig
                )),
                entry("back_to_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(Math.PI)),
                        List.of(),
                        new Pose2d(1, 0, new Rotation2d(Math.PI)),
                        trajectoryConfigReversed
                )),

                //new auton
                entry("to_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0,0, new Rotation2d(0)),
                        List.of(),
                        new Pose2d(Units.inchesToMeters(24.19),0,new Rotation2d(0)),
                        trajectoryConfig
                )),
                entry("angle_ramp_forward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0,0,new Rotation2d(0)),
                        List.of(),
                        new Pose2d(Units.inchesToMeters(4.75)+placeholderconstant,0,new Rotation2d(0)),
                        trajectoryConfigSlow
                )),
                entry("go_over_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0,0,new Rotation2d(0)),
                        List.of(),
                        new Pose2d(Units.inchesToMeters(118.02)+placeholderconstant,0,new Rotation2d(0)),
                        trajectoryConfig
                )),
                entry("angle_ramp_backward", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0,0,new Rotation2d(Math.PI)),
                        List.of(),
                        new Pose2d(Units.inchesToMeters(4.75)+placeholderconstant,0,new Rotation2d(Math.PI)),
                        trajectoryConfigSlowReversed
                )),
                entry("go_middle_ramp", TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0,0,new Rotation2d(Math.PI)),
                        List.of(),
                        new Pose2d(Units.inchesToMeters(53.5)+placeholderconstant,0,new Rotation2d(Math.PI)),
                        trajectoryConfigReversed
                ))
        );
        }
