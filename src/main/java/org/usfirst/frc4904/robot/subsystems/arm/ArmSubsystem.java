package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmSubsystem extends SubsystemBase {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 1;
    public static final double MAX_ACCEL_EXTENSION = 2;
    
    public static final double MAX_VELOCITY_PIVOT = 150;
    public static final double MAX_ACCEL_PIVOT = 200;

    
    public static final HashMap<Integer, Pair<Double, Double>> cones = new HashMap<>(); //in degrees, meters
    static {
        cones.put(1, new Pair<>(-19.,Units.inchesToMeters(0.)));
        cones.put(2, new Pair<>(29.,Units.inchesToMeters(18.)));
        cones.put(3, new Pair<>(31.,Units.inchesToMeters(38.)));
    }
    public static final HashMap<Integer, Pair<Double, Double>> cubes = new HashMap<Integer, Pair<Double, Double>>(); //in degrees, meters
    static {
        cubes.put(1, new Pair<>(-33.,Units.inchesToMeters(0.)));
        cubes.put(2, new Pair<>(14.,Units.inchesToMeters(6.)));
        cubes.put(3, new Pair<>(22.,Units.inchesToMeters(28.)));
    }

    public static final HashMap<String, Pair<Double, Double>> otherPositions = new HashMap<>();
    static {
        // https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit#gid=0 in degrees, meters
        otherPositions.put("homeUp", new Pair<>(70., Units.inchesToMeters(0.))); // TODO: get number @thomasrimer
        otherPositions.put("homeDown", new Pair<>(-37., Units.inchesToMeters(0.)));
        otherPositions.put("intakeGround", new Pair<>(-37., Units.inchesToMeters(4.)));
    }


    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_posReturnToHomeUp(boolean flippy) {
        var cmd = c_holdArmPoseFlippy(otherPositions.get("homeUp"), NathanGain.isFlippy);
        cmd.setName("arm position - home (up)");
        return cmd;
    }
    public Command c_posReturnToHomeDown(boolean flippy) {
        var cmd = c_holdArmPoseFlippy(otherPositions.get("homeDown"), NathanGain.isFlippy);
        cmd.setName("arm position - home (down)");
        return cmd;
    }
    public Command c_posIntakeGround() {
        var cmd = c_holdArmPoseFlippy(otherPositions.get("intakeGround"), NathanGain.isFlippy);
        cmd.setName("arm position - ground intake");
        return cmd;
    }
    public Command c_posIntakeShelf() {
        // TODO: back up 14 inches -- remember to always use meters
        var cmd = c_holdArmPoseFlippy(otherPositions.get("intakeShelf"), NathanGain.isFlippy);
        cmd.setName("arm position - pre shelf intake");
        return cmd;
    }

    public Command c_angleCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();

        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }

        var cmd = c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
        cmd.setName("arm - c_angleCubes - " + shelf);
        return cmd;
    }
    //for auton
    public Command placeCube(int shelf, boolean flippy) {
        double degreesFromHorizontal = cubes.get(shelf).getFirst();
        if (flippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }
        double extensionLengthMeters= cubes.get(shelf).getSecond();

        return (
            c_holdArmPose(degreesFromHorizontal, extensionLengthMeters)
            .alongWith(new WaitCommand(1).andThen(RobotMap.Component.intake.c_holdVoltage(-Intake.DEFAULT_INTAKE_VOLTS))
        )).withTimeout(5); //TODO: change timeout
    }
    //for buttons
    public Command placeCube(int shelf) {
        double degreesFromHorizontal = cubes.get(shelf).getFirst();
        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }
        double extensionLengthMeters= cubes.get(shelf).getSecond();

        return (
            c_holdArmPose(degreesFromHorizontal, extensionLengthMeters)
            .alongWith(new WaitCommand(1).andThen(RobotMap.Component.intake.c_holdVoltage(Intake.DEFAULT_INTAKE_VOLTS))
        )).withTimeout(5); //TODO: change timeout
    }
    public Command placeCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();
        
        if (NathanGain.isFlippy) {
            degreesFromHorizontal = (degreesFromHorizontal * -1) + 180;
        }

        var cmd = c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
        cmd.setName("arm - c_angleCones - " + shelf);
        return cmd;
    }

    public Command c_holdArmPoseFlippy(Pair<Double, Double> angleAndExtensionMeters, boolean flippy) {
        var degreesFromHorizontal = angleAndExtensionMeters.getFirst();
        var extensionLengthMeters = angleAndExtensionMeters.getSecond();

        if (flippy) degreesFromHorizontal = 180 - degreesFromHorizontal;

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }

    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters) {
        // TODO: crashes
        // return this.runOnce(() -> System.out.println("TODO: hold arm pose crashes the code!"));
        Command firstCommand;
        Command secondCommand;
        double wait;

        Pair<Command, Double> pivotMovement = armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT);
        Pair<Command, Double> extensionMovement = armExtensionSubsystem.c_holdExtension(extensionLengthMeters, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION);

        if ((extensionLengthMeters - armExtensionSubsystem.getCurrentExtensionLength()) > 0) {
            firstCommand = pivotMovement.getFirst();
            wait = pivotMovement.getSecond();
            secondCommand = extensionMovement.getFirst();
        } else {
            firstCommand = extensionMovement.getFirst();
            wait = extensionMovement.getSecond();
            secondCommand = pivotMovement.getFirst();
        }
        var cmd = this.runOnce(() -> {
                    firstCommand.schedule();
                    // (Commands.run(() -> System.out.println(
                    //     "1: " + String.valueOf(firstCommand.isScheduled())
                    // + " ... 2: " + String.valueOf(secondCommand.isScheduled())
                    // + " ... cur :  " + armExtensionSubsystem.getCurrentCommand().getName() 
                    // + " ... joystick: " + String.valueOf(RobotMap.HumanInput.Operator.joystick.getAxis(3))
                    //     ))).schedule();
                    (new SequentialCommandGroup(new WaitCommand(wait), secondCommand)).schedule();
        // ((new WaitCommand(1)).andThen(secondCommand)).schedule();
        // secondCommand.schedule();      
        }); // long story. basically, parallel command group requires it's subcommands' requirements. however, we want one subcommand to be able to die wihle the other one lives, so we just do this instead and leak commands. it's fine because they'll get cleaned up when their atomic base subsystems gets taken over by new commands
        cmd.setName("arm - c_holdArmPose");
        return cmd;
    }
}
