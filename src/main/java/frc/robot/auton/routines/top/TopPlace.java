// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.routines.top;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.commands.ResetLocation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.PlaceCube;
import frc.robot.subsystems.elevator.commands.ScoreCubeHigh;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopPlace extends SequentialCommandGroup {
  /** Creates a new TopPlace. */
  public TopPlace(SwerveDrivetrain drivetrain, ActiveIntake intake, Claw claw, ClawPitch clawPitch, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetLocation(drivetrain, new Pose2d(1.9, 4.4, Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue)? 0 : 180))),
      Commands.runOnce(() ->intake.setForceBeamOpen(true)),
      new ScoreCubeHigh(drivetrain, elevator, claw, clawPitch, intake),
      new PlaceCube(drivetrain, elevator, claw, clawPitch, intake),
      Commands.runOnce(() ->intake.setForceBeamOpen(false))
    );
  }
}
