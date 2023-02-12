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
import pabeles.concurrency.IntOperatorTask.Max;

public class autoBalancePitch extends CommandBase {
  SwerveDrivetrain drivetrain;
  OzoneImu pigeon;
  //safe speed: 0.01
  final double SPEED = 0.01;
  //safe tolernace = 2.0
  final double TOLERANCE = 2.0;
  double pitchSpeed = 0;
  double previousPitch = 0;
  double rollSpeed = 0;
  double pitch;

  static final double MAX_ERROR = 19;
  double PITCH_PID = 0;
  double ROLL_PID = 0;

  boolean isPitchAtSetpoint = false;
  boolean isRollAtSetpoint = false;

  //safe p: 0.00585
  PIDController pidPitch = new PIDController(0.006, 0, 0.002269);
  PIDController pidRoll = new PIDController(0.00585, 0, 0);

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
    pidPitch.setTolerance(TOLERANCE);
    pidPitch.setSetpoint(0);
    pidRoll.setTolerance(TOLERANCE);
    pidRoll.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = pigeon.getPitch();
    PITCH_PID = pidPitch.calculate(MathUtil.clamp(pigeon.getPitch(), pidPitch.getSetpoint() - MAX_ERROR, pidPitch.getSetpoint() + MAX_ERROR));
    ROLL_PID = pidRoll.calculate(MathUtil.clamp(pigeon.getRoll(), pidRoll.getSetpoint() - MAX_ERROR, pidRoll.getSetpoint() + MAX_ERROR));
    ChassisSpeeds speed = new ChassisSpeeds(PITCH_PID, ROLL_PID, 0);
    if(!(pidPitch.atSetpoint() && pidRoll.atSetpoint())) {
      drivetrain.drive(speed, false);
    }
    else{
      ChassisSpeeds stopDrive = new ChassisSpeeds(0, 0, 0);
      drivetrain.drive(stopDrive, false);
    }

    previousPitch = pitch;

    isPitchAtSetpoint = pidPitch.atSetpoint();
    isRollAtSetpoint = pidRoll.atSetpoint();

    SmartDashboard.putNumber("PID Pitch Output", PITCH_PID);
    SmartDashboard.putNumber("PID Roll Output", ROLL_PID);
    SmartDashboard.putBoolean("Is Pitch Ar Setpoint", isPitchAtSetpoint);
    SmartDashboard.putBoolean("Is Roll At Setpoint", isRollAtSetpoint);

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
