// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.activeintake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.activeintake.ActiveIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseIntake extends InstantCommand {
  private ActiveIntake intake;

  public ReverseIntake(ActiveIntake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setForceBelts(true);
    intake.setUpperMotor(-ActiveIntake.UPPER_MOTOR_SPEED);
    intake.setLowerMotor(-ActiveIntake.LOWER_MOTOR_CONE_SPEED);
  }
}
