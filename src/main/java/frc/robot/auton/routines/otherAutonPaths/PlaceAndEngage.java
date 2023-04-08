// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.routines.otherAutonPaths;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.auton.AutonPaths;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.RetractIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveElevator;
import frc.robot.subsystems.elevator.commands.PlaceCube;
import frc.robot.subsystems.elevator.commands.ScoreCubeHigh;
import frc.robot.telemetry.commands.AutoBalance;
import frc.robot.telemetry.commands.DriveOntoChargeStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAndEngage extends SequentialCommandGroup {
  /** Creates a new BottomTaxi. */  
  public PlaceAndEngage(ActiveIntake intake, SwerveDrivetrain drivetrain, Claw claw, ClawPitch clawPitch, Elevator elevator, AutonPaths paths) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.runOnce(() -> {
          SubsystemManager.getInstance().getImu().resetPitch();
          SubsystemManager.getInstance().getImu().resetRoll();
          SubsystemManager.getInstance().getImu().reset();
        }),
        new ScoreCubeHigh(drivetrain, elevator, claw, clawPitch, intake),
        new PlaceCube(drivetrain, elevator, claw, clawPitch, intake),
        new ParallelCommandGroup(new MoveElevator(elevator, 0.9), new RetractIntake(intake), new DriveOntoChargeStation(drivetrain)),
        new AutoBalance(drivetrain)
    );
  }
}