// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.auton.commands.DriveBack;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.PlaceCone;
import frc.robot.subsystems.elevator.commands.ScoreConeHigh;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeAndGoBack extends SequentialCommandGroup {
  /** Creates a new PlaceConeAndGoBack. */
  public PlaceConeAndGoBack(SwerveDrivetrain drivetrain, ActiveIntake intake, Claw claw, ClawPitch clawPitch, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ProxyCommand(() -> Commands.runOnce(() -> SubsystemManager.getInstance().getImu().setReset(Rotation2d.fromDegrees((DriverStation.getAlliance() == Alliance.Blue)? 0 : 180)))),
      new ScoreConeHigh(drivetrain, elevator, claw, clawPitch, intake),
      new PlaceCone(drivetrain, elevator, claw, clawPitch, intake),
      new DriveBack(drivetrain, 4.4)
    );
  }
}
