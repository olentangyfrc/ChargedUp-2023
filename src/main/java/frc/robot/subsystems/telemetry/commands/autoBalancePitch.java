// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telemetry.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.telemetry.OzoneImu;

public class autoBalancePitch extends CommandBase {
  SwerveDrivetrain drivetrain;
  OzoneImu pigeon;
  final double SPEED = .01;
  final double TOLERANCE = 2.0;
  double pitchSpeed = 0;
  double previousPitch = 0;
  double rollSpeed = 0;
  double pitch;

  double PITCH_PID = 0;
  double ROLL_PID = 0;

  PIDController pid = new PIDController(0.0061, 0, 0);

  /** Creates a new autoBalance. */
  public autoBalancePitch(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    pigeon = SubsystemManager.getInstance().getImu();
    addRequirements(drivetrain);
  }

  /*public double bangBang(double pitch, double tolerance) {
    return Math.signum(MathUtil.applyDeadband(pitch, tolerance));
  }
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitch = pigeon.getPitch();
    pid.setTolerance(TOLERANCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = pigeon.getPitch();
    PITCH_PID = pid.calculate(pigeon.getPitch());
    ROLL_PID = pid.calculate(pigeon.getRoll());
    ChassisSpeeds speed = new ChassisSpeeds(pid.calculate(pigeon.getPitch()), pid.calculate(pigeon.getRoll()), 0);
    drivetrain.drive(speed, false);

    previousPitch = pitch;

    SmartDashboard.putNumber("PID Pitch Output", PITCH_PID);
    SmartDashboard.putNumber("PID Roll Output", ROLL_PID);

    //SmartDashboard.putNumber("Bang Bang Output", bangBang(pitch, TOLERANCE));

    /*if (bangBang(pitch, TOLERANCE) != 0) {
      pitchSpeed = SPEED * bangBang(pitch, TOLERANCE);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return bangBang(pitch, TOLERANCE) == 0;
  }
}
