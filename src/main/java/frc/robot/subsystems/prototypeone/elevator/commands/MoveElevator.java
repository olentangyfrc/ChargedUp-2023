// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.prototypeone.elevator.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.prototypeone.elevator.ProtoElevator;

public class MoveElevator extends CommandBase {
  private ProtoElevator elevator;
  private double targetPosition;

  private GenericEntry elevatorPosition = Shuffleboard.getTab("Elevator").add("Elevator Target Position", 0).getEntry();

  /** Creates a new MoveElevator. */
  public MoveElevator(ProtoElevator elevator, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    // this.targetPosition = targetPosition;
    this.targetPosition = targetPosition;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPosition = elevatorPosition.getDouble(0);
    elevator.setTargetPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.isAtTargetPosition();
  }
}