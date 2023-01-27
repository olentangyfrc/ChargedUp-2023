// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.telemetry.OzoneImu;
import frc.robot.subsystems.telemetry.Pigeon2;

public class autoBalance extends CommandBase {
  SwerveDrivetrain drivetrain;
  OzoneImu pigeon;

  /** Creates a new autoBalance. */
  public autoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain = SubsystemManager.getInstance().getDrivetrain();
    pigeon = SubsystemManager.getInstance().getImu();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PIDController pid = new PIDController(0, 0, 0);
    ChassisSpeeds speed = new ChassisSpeeds(pid.calculate(pigeon.getPitch()), pid.calculate(pigeon.getRoll()), 0);
    drivetrain.drive(speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
