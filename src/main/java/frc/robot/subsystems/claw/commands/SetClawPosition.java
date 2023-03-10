// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetClawPosition extends InstantCommand {
  private Claw claw;
  private ClawPosition clawPosition;

  public SetClawPosition(Claw claw, ClawPosition clawPosition) {
    this.claw = claw;
    this.clawPosition = clawPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setClawPosition(clawPosition);
    if(clawPosition == ClawPosition.LOWER_LATCH || clawPosition == ClawPosition.UPPER_LATCH || clawPosition == ClawPosition.OPEN) {
      if(SubsystemManager.getInstance().getActiveIntake() != null) {
        SubsystemManager.getInstance().getActiveIntake().setClawHoldingGamePiece(false);
      }
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("Claw position set");

  }
}
