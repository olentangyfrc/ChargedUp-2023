// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.RotateToAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoBalancePitchGroup extends SequentialCommandGroup {
  /** Creates a new autoBalancePitchGroup. */
  public autoBalancePitchGroup(/*Rotation2d rotateAngle*/) {
    SwerveDrivetrain drivetrain = SubsystemManager.getInstance().getDrivetrain();
    //addCommands(new RotateToAngle(drivetrain, rotateAngle));
    addCommands(new driveUp(drivetrain));
    addCommands(new autoBalancePitch(drivetrain));
  }
}
