package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmSubsystem {
    public final ArmPivotSubsystem armPivotSubsystem;
    public final ArmExtensionSubsystem armExtensionSubsystem;
    public ArmSubsystem(ArmPivotSubsystem pivotArmSubsystem, ArmExtensionSubsystem armExtensionSubsystem) {
        this.armPivotSubsystem = pivotArmSubsystem;
        this.armExtensionSubsystem = armExtensionSubsystem;
    }
    
    public ParallelCommandGroup c_holdArmPose(double degreesFromHorizontal, double extensionLengthInches) {
        var cmdgrp = new ParallelCommandGroup(
            armPivotSubsystem.c_holdRotation(degreesFromHorizontal),
            armExtensionSubsystem.c_holdExtension(extensionLengthInches)
        );
        cmdgrp.addRequirements(armPivotSubsystem, armExtensionSubsystem);
        return cmdgrp;
    }


}
