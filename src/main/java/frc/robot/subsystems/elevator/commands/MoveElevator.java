// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MoveElevator extends CommandBase {
  private Elevator elevator;
  private ElevatorPosition positionPreset;
  private double position;

  /** Creates a new MoveElevator. */
  public MoveElevator(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
  }

  /** Creates a new MoveElevator. */
  public MoveElevator(Elevator elevator, ElevatorPosition position) {
    this.elevator = elevator;
    this.positionPreset = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(positionPreset != null) {
      elevator.setTargetPosition(positionPreset);
    } else {
      elevator.setTargetPosition(position);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtTargetPosition();
  }
}
