// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeArm.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeArm.intakeArm;
import frc.robot.subsystems.elevator.commands.MoveElevator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PlaceItem extends SequentialCommandGroup {
  
  //item 1: cone, item 2: cube
  //height 1: low, height2: cube
  
  private intakeArm intakeArm = SubsystemManager.getInstance().getIntakeArm();
  private Elevator elevator = SubsystemManager.getInstance().getElevator();
  
  public PlaceItem(String item, String height) {
    if (item.equalsIgnoreCase("Cone")) {
      if (height.equalsIgnoreCase("High") || height.equalsIgnoreCase("Low")) {
        PlaceCone();
      } else {
        System.out.println("Invalid input to PlaceItem. Expected values \"Low\" or \"High\" for height but received " + height);
      }

    } else if (item.equalsIgnoreCase("Cube")) {

      if (height.equalsIgnoreCase("High")) {
        PlaceCubeHigh();
      } else if (height.equalsIgnoreCase("Low")) {
        PlaceCubeLow();
      } else {
        System.out.println("Invalid input to PlaceItem. Expected values \"Low\" or \"High\" for height but received " + height);
      }

    } else {
      System.out.println("Invalid input to PlaceItem. Expected values \"Cone\" or \"Cube\" for item but received " + item);
    }
  }

  public void ReleaseItem() {
    addCommands(
      new SequentialCommandGroup(
        new WaitCommand(1.0),
        new armDown(intakeArm),
        new WaitCommand(2.0),
        new toggleClaw(intakeArm),
        new WaitCommand(0.3),
        new armUp(intakeArm)
      )
    );
  }

  public void PlaceCubeLow() {
    addCommands(
      new MoveElevator(elevator, 1) //Still waiting on a middle value
    );

    ReleaseItem();
  }

  public void PlaceCubeHigh() {
    addCommands(
      new MoveElevator(elevator, Elevator.ELEVATOR_HIGH_POS)
    );

    ReleaseItem();
  }

  public void PlaceCone() {
    addCommands(
      new MoveElevator(elevator, Elevator.ELEVATOR_HIGH_POS)
    );

    ReleaseItem();
  }
}