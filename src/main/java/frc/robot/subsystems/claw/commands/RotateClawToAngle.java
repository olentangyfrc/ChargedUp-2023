// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.claw.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateClawToAngle extends InstantCommand {
  private Claw claw;
  private Rotation2d targetAngle;

  public RotateClawToAngle(Claw claw, Rotation2d targetAngle) {
    this.claw = claw;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setTargetClawAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) { 
    System.out.println("CLAW ANGLE FINISHED");   
    claw.stopWristMotor();
  }

  @Override
  public boolean isFinished() {
    return claw.isWristAtSetpoint();
  }
}