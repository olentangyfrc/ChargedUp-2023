// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeployElevator extends InstantCommand {
  private Elevator e;

  public DeployElevator(Elevator e) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.e = e;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    e.moveElevatorForward();
    System.out.println("DeployElevator.initialize()");
  }
}
