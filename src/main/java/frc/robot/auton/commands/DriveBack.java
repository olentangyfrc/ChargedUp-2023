// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class DriveBack extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private double duration;
  private double startTime;


  /** Creates a new DriveBack. */
  public DriveBack(SwerveDrivetrain drivetrain, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.duration = duration;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setIsFollowingPath(true);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(1, 0, 0), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    drivetrain.setIsFollowingPath(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= duration;
  }
}
