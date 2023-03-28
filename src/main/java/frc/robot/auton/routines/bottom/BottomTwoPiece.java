// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.routines.bottom;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auton.AutonPaths;
import frc.robot.auton.AutonPaths.AutoTrajectory;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.PlaceCone;
import frc.robot.subsystems.elevator.commands.PlaceCube;
import frc.robot.subsystems.elevator.commands.ScoreConeHigh;
import frc.robot.subsystems.elevator.commands.ScoreCubeHigh;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomTwoPiece extends SequentialCommandGroup {
  /** Creates a new TopTwoPiece. */
  public BottomTwoPiece(ActiveIntake intake, Claw claw, ClawPitch clawPitch, Elevator elevator, AutonPaths paths) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() ->intake.setForceBeamOpen(true)),
      new ScoreCubeHigh(elevator, claw, clawPitch, intake),
      new PlaceCube(elevator, claw, clawPitch, intake),
      Commands.runOnce(() ->intake.setForceBeamOpen(false)),
      new ProxyCommand( () -> paths.followTrajectoryCommand(paths.getTrajectory(AutoTrajectory.GetGamepieceFour))),
      new WaitUntilCommand(() -> intake.isClawHoldingGamePiece() && !intake.isGrabbing()),
      new ScoreConeHigh(elevator, claw, clawPitch, intake),
      new PlaceCone(elevator, claw, clawPitch, intake)
    );
  }
}
