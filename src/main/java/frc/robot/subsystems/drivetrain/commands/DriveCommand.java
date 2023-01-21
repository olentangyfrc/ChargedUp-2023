// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.commands;

import java.util.Map;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

/**
 * This command will grab user input from IO and call the drivetrain's drive method.
 * This should be called periodically by the drivetrain.
 */
public class DriveCommand extends CommandBase {
  private SwerveDrivetrain drivetrain;
  private GenericEntry speedEntry = Shuffleboard.getTab("Drive").add("Drive percent", 1).withProperties(Map.of("min", -1, "max", 1)).getEntry();

  public DriveCommand(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  public void execute() {
    IO io = IO.getInstance();
    ChassisSpeeds speeds = new ChassisSpeeds(
      io.getLeftY() * SwerveDrivetrain.MAX_LINEAR_SPEED * speedEntry.getDouble(1),
      -io.getLeftX() * SwerveDrivetrain.MAX_LINEAR_SPEED * speedEntry.getDouble(1),
      -io.getRightX() * SwerveDrivetrain.MAX_ROTATION_SPEED * speedEntry.getDouble(1)
    );
    
    if(!drivetrain.getIsFollowingPath()) {
      drivetrain.drive(speeds, true);
    }

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
