package org.usfirst.frc4904.robot.subsystems.arm;

import org.usfirst.frc4904.standard.subsystems.RequirementsSubsystemBase;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ArmSubsystem extends RequirementsSubsystemBase {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;

    public static final double MAX_VELOCITY_EXTENSION = 0;
    public static final double MAX_ACCEL_EXTENSION = 0;
    
    public static final double MAX_VELOCITY_PIVOT = 0;
    public static final double MAX_ACCEL_PIVOT = 0;

    public ArmSubsystem(ArmPivotSubsystem armPivotSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        super(armPivotSubsystem, armExtensionSubsystem);
        this.armPivotSubsystem = armPivotSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }

    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        Command firstCommand;
        Command secondCommand;
        double wait;

        Pair<Command, Double> pivotMovement = armPivotSubsystem.c_holdRotation(degreesFromHorizontal, MAX_VELOCITY_PIVOT, MAX_ACCEL_PIVOT);
        Pair<Command, Double> extensionMovement = armExtensionSubsystem.c_holdExtension(extensionLengthInches, MAX_VELOCITY_EXTENSION, MAX_ACCEL_EXTENSION);

        if ((extensionLengthInches - armExtensionSubsystem.getCurrentExtensionLength()) > 0) {
            firstCommand = pivotMovement.getFirst();
            wait = pivotMovement.getSecond();
            secondCommand = extensionMovement.getFirst();
        } else {
            firstCommand = extensionMovement.getFirst();
            wait = extensionMovement.getSecond();
            secondCommand = pivotMovement.getFirst();
        }

        var cmdgrp = new ParallelCommandGroup(
            firstCommand,
            new SequentialCommandGroup(new WaitCommand(wait * 1000), secondCommand)
        );
        cmdgrp.addRequirements(this);
        return cmdgrp;
    }


}
