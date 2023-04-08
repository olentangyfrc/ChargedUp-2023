// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.DeployIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeMiddle extends SequentialCommandGroup {
  /** Creates a new ScoreMiddle. */
  public ScoreConeMiddle(SwerveDrivetrain drivetrain, Elevator e, Claw c, ClawPitch cp, ActiveIntake ai) {
    addCommands(
        new ParallelCommandGroup(
            Commands.runOnce(() -> drivetrain.setSpeedPercent(SwerveDrivetrain.PLACE_SPEED_PERCENT)),
            new DeployIntake(ai),
            // new WaitCommand(.25),
            new RotateClawToAngle(c, Rotation2d.fromDegrees(180)),
            new RotateClawPitch(cp, Rotation2d.fromDegrees(115))),
            new ParallelCommandGroup(
              new DeployElevator(e),
              new SequentialCommandGroup(
                new WaitCommand(0.2),
                new MoveElevator(e, ElevatorPosition.MIDDLE)
              )
            )
    );
  }
}
