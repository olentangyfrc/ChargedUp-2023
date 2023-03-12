// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.DeployIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.commands.RotateClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;
import frc.robot.subsystems.claw.commands.SetClawPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeHigh extends SequentialCommandGroup {
  /** Creates a new ScoreMiddle. */
  public ScoreConeHigh(Elevator e, Claw c, ClawPitch cp, ActiveIntake ai) {
    addCommands(
        new DeployIntake(ai),
        // new WaitCommand(.25),
        new RotateClawToAngle(c, Rotation2d.fromDegrees(180)),
        new RotateClawPitch(cp, Rotation2d.fromDegrees(115)),
        new ParallelCommandGroup(
            new MoveElevator(e, ElevatorPosition.HIGH),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new DeployElevator(e))),
        new WaitCommand(1.5),
        new ParallelCommandGroup(
            new SetClawPosition(c, ClawPosition.LOWER_LATCH),
            new MoveElevator(e, ElevatorPosition.LOW),
            new SequentialCommandGroup(
                new WaitCommand(.25),
                new RetractElevator(e))));
  }
}