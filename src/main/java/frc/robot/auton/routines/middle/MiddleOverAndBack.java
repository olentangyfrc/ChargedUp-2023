// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.routines.middle;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.auton.AutonPaths;
import frc.robot.auton.AutonPaths.AutoTrajectory;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.PlaceCube;
import frc.robot.subsystems.elevator.commands.ScoreCubeHigh;
import frc.robot.telemetry.commands.AutoBalance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleOverAndBack extends SequentialCommandGroup {
  /** Creates a new MiddleOverAndBack. */
  public MiddleOverAndBack(ActiveIntake intake, SwerveDrivetrain drivetrain, Claw claw, ClawPitch clawPitch, Elevator elevator, AutonPaths paths) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> {
        SubsystemManager.getInstance().getImu().resetPitch();
        SubsystemManager.getInstance().getImu().resetRoll();
      }),
      Commands.runOnce(() ->intake.setForceBeamOpen(true)),
      new ScoreCubeHigh(drivetrain, elevator, claw, clawPitch, intake),
      new PlaceCube(drivetrain, elevator, claw, clawPitch, intake),
      Commands.runOnce(() ->intake.setForceBeamOpen(false)),
      new ProxyCommand(() -> paths.followTrajectoryCommand(paths.getTrajectory(AutoTrajectory.OverAndBack)).andThen(new AutoBalance(drivetrain)))
    );
  }
}
