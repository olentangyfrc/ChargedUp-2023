// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.claw.Claw;

public class ManualClawBackwards extends CommandBase {
  private Claw claw;

  /** Creates a new ManualClawForwards. */
  public ManualClawBackwards(Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setIsManualControl(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setClawVoltage(-Claw.MANUAL_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopWristMotor();
    claw.setTargetClawAngle(claw.getWristAngle());
    claw.setIsManualControl(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
