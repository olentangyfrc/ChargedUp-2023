// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;
import frc.robot.subsystems.claw.commands.SetClawPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCone extends SequentialCommandGroup {
  /** Creates a new PlaceCone. */
  public PlaceCone(Elevator e, Claw c, ClawPitch cp, ActiveIntake ai) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ProxyCommand(() -> new MoveElevator(e, e.getPosition() - 0.75)),
        new SequentialCommandGroup(
          new WaitCommand(0.2),
          new SetClawPosition(c, ClawPosition.UPPER_LATCH),
          new RetractElevator(e),
          new WaitCommand(0.4),
          new MoveElevator(e, ElevatorPosition.LOW)
        )
        // new SequentialCommandGroup(
        // )
      ),
      new ScheduleCommand(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new SetClawPosition(c, ClawPosition.CLOSED),
            new RotateClawPitch(cp, Rotation2d.fromDegrees(0))
          ),
          new RotateClawToAngle(c, Rotation2d.fromDegrees(0))
        )
      )
      // new ParallelCommandGroup(
      //       new SequentialCommandGroup(
      //       ),
      //       new SequentialCommandGroup(
      //           new WaitCommand(0.25),
      // new WaitCommand(0.25),

    );
  }
}
