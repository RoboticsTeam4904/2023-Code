package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.standard.commands.WaitUntil;
import org.usfirst.frc4904.standard.custom.Triple;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class ArmSubsystem extends SubsystemBase {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 1.5;
    public static final double MAX_ACCEL_EXTENSION = 3;
    
    public static final double MAX_VELOCITY_PIVOT = 160;
    public static final double MAX_ACCEL_PIVOT = 210;

    
    // SHELF CONES
    public static final HashMap<Integer, Triple<Double, Double, Double>> shelfCones = new HashMap<>(); //in degrees, meters
    static { // SHELF CONES
        // cones.put(1, new Triple<>(-19., Units.inchesToMeters(0), 3.));
        shelfCones.put(2, new Triple<>(29.5, Units.inchesToMeters(18), 3.));
        shelfCones.put(3, new Triple<>(41., ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));
        shelfCones.put(4, new Triple<>(180.0-41, ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));
    }
    
    // FLOOR CONES
    public static final HashMap<Integer, Triple<Double, Double, Double>> floorCones = new HashMap<>(); //in degrees, meters
    static { // FLOOR CONES
        // cones.put(1, new Triple<>(-19., Units.inchesToMeters(0), 3.));
        floorCones.put(2, new Triple<>(29., Units.inchesToMeters(14), 3.));
        floorCones.put(3, new Triple<>(29., ArmExtensionSubsystem.MAX_EXTENSION_M-0.02, 3.));
        floorCones.put(4, new Triple<>(180.0-41, ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));
    }

    public static HashMap<Integer, Triple<Double, Double, Double>> cones = floorCones;

    public static final HashMap<Integer, Triple<Double, Double, Double>> cubes = new HashMap<>(); //in degrees, meters
    static {
        // cubes.put(1, new Triple<>(-33., Units.inchesToMeters(0), 3.));
        cubes.put(2, new Triple<>(15., Units.inchesToMeters(0), 4.5));
        cubes.put(3, new Triple<>(20., Units.inchesToMeters(0), 4.5));
        cubes.put(4, new Triple<>(180.-35, Units.inchesToMeters(0), 4.5));
        cubes.put(5, new Triple<>(180.-25, Units.inchesToMeters(0), 4.5));
    }

    public static final HashMap<String, Pair<Double, Double>> otherPositions = new HashMap<>();
    static {
        // https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit#gid=0 in degrees, meters
        otherPositions.put("homeUp", new Pair<>(65., Units.inchesToMeters(0.))); // TODO: get number @thomasrimer
        otherPositions.put("homeDown", new Pair<>(-41., -0.1));
        otherPositions.put("intakeShelf", new Pair<>(25., Units.inchesToMeters(20.)));
    }


    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_posReturnToHomeUp() {
        var cmd = c_holdArmPose(otherPositions.get("homeUp").getFirst(), otherPositions.get("homeUp").getSecond());
        cmd.setName("arm position - home (up)");
        return cmd;
    }

    public Command c_posReturnToHomeDown() {
        var cmd = c_holdArmPose(otherPositions.get("homeDown").getFirst(), otherPositions.get("homeDown").getSecond());
        cmd.setName("arm position - home (down)");
        return cmd;
    }
    public Command c_posIntakeShelf() {
        // TODO: back up 14 inches -- remember to always use meters
        cones = shelfCones;
        var cmd = c_holdArmPose(otherPositions.get("intakeShelf").getFirst(), otherPositions.get("intakeShelf").getSecond());
        cmd.setName("arm position - pre shelf intake");
        return cmd;
    }
    public Command c_posIntakeFloor() {
        cones = floorCones;
        var cmd = c_holdArmPose(otherPositions.get("homeDown").getFirst(), otherPositions.get("homeDown").getSecond());
        cmd.setName("arm position - pre floor intake");
        return cmd;
    }

    public Command c_angleCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }

    public Command c_shootCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();
        var voltage = cones.get(shelf).getThird();


        if (cones == shelfCones) {
            return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
        }

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters)
        .alongWith((new WaitUntilCommand(() -> armExtensionSubsystem.isArmAtExtension(extensionLengthMeters) && armPivotSubsystem.isArmAtRotation(degreesFromHorizontal)))
        .andThen(RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5))
        .andThen(RobotMap.Component.intake.c_neutralOutput())
        .andThen(c_posReturnToHomeUp())
        );
    }
   
    public Command c_angleCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }


    public Command c_shootCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();
        var voltage = cubes.get(shelf).getThird();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters)
        .alongWith((new WaitUntilCommand(() -> armExtensionSubsystem.isArmAtExtension(extensionLengthMeters) && armPivotSubsystem.isArmAtRotation(degreesFromHorizontal)))
        .andThen(RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5))
        .andThen(RobotMap.Component.intake.c_neutralOutput())
        .andThen(c_posReturnToHomeUp())
        );
    }

    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters) {        
        if (extensionLengthMeters > armExtensionSubsystem.getCurrentExtensionLength()) {
            return armExtensionSubsystem.c_holdExtension(extensionLengthMeters, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION).alongWith(
               new WaitUntilCommand(() -> armExtensionSubsystem.isArmAtExtension(extensionLengthMeters))
               .andThen(armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT))
            );
        } else {
            return armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT).alongWith(
                new WaitUntilCommand(() -> armPivotSubsystem.isArmAtRotation(degreesFromHorizontal))
                .andThen(armExtensionSubsystem.c_holdExtension(extensionLengthMeters, MAX_VELOCITY_EXTENSION, MAX_VELOCITY_PIVOT))
            );
            // return armPivotSubsystem.c_holdRotation(degreesFromHorizontal).withTimeout(1).andThen(armExtensionSubsystem.c_holdExtension(extensionLengthMeters));
        }
    }
}
