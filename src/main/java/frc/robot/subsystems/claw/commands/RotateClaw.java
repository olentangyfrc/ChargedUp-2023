// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;

public class RotateClaw extends CommandBase {
  private Claw claw;
  private Rotation2d wristAngle;

  /** Creates a new RotateClaw. */
  public RotateClaw(Claw claw, Rotation2d wristAngle) {
    this.claw = claw;
    this.wristAngle = wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setTargetWristAngle(wristAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.isWristAtAngle();
  }
}
