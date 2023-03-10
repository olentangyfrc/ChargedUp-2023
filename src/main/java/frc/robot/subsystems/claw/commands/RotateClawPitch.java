// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawPitch;

public class RotateClawPitch extends CommandBase {
  private ClawPitch claw;
  private Rotation2d pitchAngle;

  /** Creates a new RotateClawPitch. */
  public RotateClawPitch(ClawPitch claw, Rotation2d pitchAngle) {
    this.claw = claw;
    this.pitchAngle = pitchAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setTargetPitch(pitchAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.isAtPitch();
  }
}
