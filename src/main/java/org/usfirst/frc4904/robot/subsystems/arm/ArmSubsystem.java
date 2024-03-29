package org.usfirst.frc4904.robot.subsystems.arm;

import java.util.HashMap;
import java.util.concurrent.Callable;
import java.util.function.Function;
import java.util.function.Supplier;

import org.usfirst.frc4904.robot.Robot;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.subsystems.Intake;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.custom.Triple;
import org.usfirst.frc4904.robot.humaninterface.drivers.NathanGain;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


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

        shelfCones.put(6, new Triple<>(180.0-41, ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));
    }
    
    // FLOOR CONES
    public static final HashMap<Integer, Triple<Double, Double, Double>> floorCones = new HashMap<>(); //in degrees, meters
    static { // FLOOR CONES
        // cones.put(1, new Triple<>(-19., Units.inchesToMeters(0), 3.));
        floorCones.put(2, new Triple<>(29., Units.inchesToMeters(14), 3.));
        floorCones.put(3, new Triple<>(29., ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));

        floorCones.put(6, new Triple<>(180.0-41, ArmExtensionSubsystem.MAX_EXTENSION_M, 3.));
    }

    public static HashMap<Integer, Triple<Double, Double, Double>> cones = floorCones;

    public static final HashMap<Integer, Triple<Double, Double, Double>> cubes = new HashMap<>(); //in degrees, meters
    static {
        // cubes.put(1, new Triple<>(-33., Units.inchesToMeters(0), 3.));
        cubes.put(2, new Triple<>(15., Units.inchesToMeters(0), 4.5));
        cubes.put(3, new Triple<>(20., Units.inchesToMeters(0), 4.5));

        cubes.put(5, new Triple<>(180.-25, Units.inchesToMeters(0), 4.5));
        cubes.put(6, new Triple<>(180.-40, Units.inchesToMeters(0), 5.));
    }

    public static final HashMap<String, Pair<Double, Double>> otherPositions = new HashMap<>();
    static {
        // https://docs.google.com/spreadsheets/d/1B7Ie4efOpuZb4UQsk8lHycGvi6BspnF74DUMLmiKGUM/edit#gid=0 in degrees, meters
        otherPositions.put("homeUp", new Pair<>(65., Units.inchesToMeters(0.)));
        otherPositions.put("homeDown", new Pair<>(-41., -0.1));
        otherPositions.put("intakeShelf", new Pair<>(25., Units.inchesToMeters(20.)));
    }


    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public Command c_posReturnToHomeUp() { return c_posReturnToHomeUp(null); }
    public Command c_posReturnToHomeUp(Supplier<Command> onArrivalCommandDealer) {
        var cmd =  c_holdArmPose(otherPositions.get("homeUp"), onArrivalCommandDealer);
        cmd.setName("arm position - home (up)");
        return cmd;
    }

    public Command c_posReturnToHomeDown() { return c_posReturnToHomeDown(null); }
    public Command c_posReturnToHomeDown(Supplier<Command> onArrivalCommandDealer) {
        var cmd = c_holdArmPose(otherPositions.get("homeDown"), onArrivalCommandDealer);
        cmd.setName("arm position - home (down)");
        return cmd;
    }
    public Command c_posIntakeShelf(Supplier<Command> onArrivalCommandDealer) {
        cones = shelfCones;
        var cmd = c_holdArmPose(otherPositions.get("intakeShelf"), onArrivalCommandDealer);
        cmd.setName("arm position - pre shelf intake");
        return cmd;
    }
    public Command c_posIntakeFloor(Supplier<Command> onArrivalCommandDealer) {
        cones = floorCones;
        var cmd = c_holdArmPose(otherPositions.get("homeDown"), onArrivalCommandDealer);
        cmd.setName("arm position - pre floor intake");
        return cmd;
    }

    public Command c_angleCones(int shelf) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters);
    }

    public Command c_shootCones(int shelf, boolean rush) { return c_shootCones(shelf, rush, null); }
    public Command c_shootCones(int shelf) { return c_shootCones(shelf, null); }
    public Command c_shootCones(int shelf, Supplier<Command> onArrivalCommandDealer) {
        return c_shootCones(shelf, false, onArrivalCommandDealer);
    }
    public Command c_shootCones(int shelf, boolean rush, Supplier<Command> onArrivalCommandDealer) {
        var degreesFromHorizontal = cones.get(shelf).getFirst();
        var extensionLengthMeters = cones.get(shelf).getSecond();
        var voltage = cones.get(shelf).getThird();

        if (cones == shelfCones || (shelf == 3 || shelf == 6)) {
            return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters, rush);
        }

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters,
            () -> new SequentialCommandGroup(
                RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5),
                RobotMap.Component.intake.c_neutralOutput(), c_posReturnToHomeUp(onArrivalCommandDealer)
            ) , true
        );
    }
   
    public Command c_angleCubes(int shelf) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters, null);
    }

    public Command c_shootCubes(int shelf) { return c_shootCubes(shelf, null); }
    public Command c_shootCubes(int shelf, Supplier<Command> onArrivalCommandDealer) {
        var degreesFromHorizontal = cubes.get(shelf).getFirst();
        var extensionLengthMeters = cubes.get(shelf).getSecond();
        var voltage = cubes.get(shelf).getThird();

        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters,
            () -> new SequentialCommandGroup(
                    RobotMap.Component.intake.c_holdVoltage(voltage).withTimeout(0.5),
                    RobotMap.Component.intake.c_neutralOutput(), c_posReturnToHomeUp(onArrivalCommandDealer)
            )
        );
    }

    public Command c_holdArmPose(Pair<Double, Double> emacs) {
        return c_holdArmPose(emacs.getFirst(), emacs.getSecond(), false);
    }
    public Command c_holdArmPose(double degrees, double extension) {
        return c_holdArmPose(degrees, extension, false);
    }
    public Command c_holdArmPose(Pair<Double, Double> emacs, boolean rush) {
        return c_holdArmPose(emacs.getFirst(), emacs.getSecond(), rush);
    }
    public Command c_holdArmPose(Pair<Double, Double> emacs, Supplier<Command> onArrivalCommandDealer) {
        return c_holdArmPose(emacs.getFirst(), emacs.getSecond(), onArrivalCommandDealer);
    }
    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters, boolean rush) {
        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters, null);
    }
    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters, Supplier<Command> onArrivalCommandDealer) {
        return c_holdArmPose(degreesFromHorizontal, extensionLengthMeters, onArrivalCommandDealer, false);
    }

    public Command c_holdArmPose(double degreesFromHorizontal, double extensionLengthMeters, Supplier<Command> onArrivalCommandDealer, boolean rush) {
        return new InstantCommand() {   // FIXME: replace with the new ParentCommandGroup / CommandBased thing
            @Override
            public void initialize() {
                final Supplier<Command> sanitizedArrivalCommandDealer = onArrivalCommandDealer != null ? onArrivalCommandDealer : () -> new Noop();

                Function<Supplier<Command>, Command> pivotAndThen = (then) -> armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT, rush, then); // FIXME: rush will cause bug if we extend before pivot (if target extension is less than current extension)
                Function<Supplier<Command>, Command> extendAndThen = (then) -> armExtensionSubsystem.c_holdExtension(extensionLengthMeters, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION, then);

                if (extensionLengthMeters > armExtensionSubsystem.getCurrentExtensionLength()) {
                    pivotAndThen.apply(() -> extendAndThen.apply(sanitizedArrivalCommandDealer)).schedule();
                } else {
                    extendAndThen.apply(() -> pivotAndThen.apply(sanitizedArrivalCommandDealer)).schedule();
                }
            }
        };
    }
}
