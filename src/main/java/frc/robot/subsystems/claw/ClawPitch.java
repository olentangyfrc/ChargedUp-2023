// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPitch extends SubsystemBase {
  private static final double GEAR_RATIO = 100;
  private static final double MAX_ERROR = 45; // Pitch error will be clamped to [-MAX_ERROR, MAX_ERROR] in degrees.
  private static final double PITCH_TOLERANCE = 0.5;

  private CANSparkMax pitchMotor;

  private PIDController pitchController = new PIDController(0.14853, 0, 0.013545);
  // private PIDController pitchController = new PIDController(0.03, 0, 0);

  private Rotation2d targetPitch = new Rotation2d();

  /** Creates a new ClawPitch. */
  public ClawPitch(int motorCanId) {
    pitchMotor = new CANSparkMax(motorCanId, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setInverted(true);
    pitchMotor.setIdleMode(IdleMode.kBrake);

    // pitchMotor.getEncoder().setPosition(0);

    pitchController.setTolerance(PITCH_TOLERANCE);

    setTargetPitch(Rotation2d.fromDegrees(45));

    Shuffleboard.getTab("Claw").addNumber("Pitch", () -> getPitch().getDegrees());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromRotations(pitchMotor.getEncoder().getPosition() / GEAR_RATIO);
  }

  public boolean isAtPitch() {
    return pitchController.atSetpoint();
  }

  public void setTargetPitch(Rotation2d pitch) {
    targetPitch = pitch;
  }

  private GenericEntry setPitch = Shuffleboard.getTab("Claw").add("Set Pitch", 0).getEntry();

  @Override
  public void periodic() {
    double targetDegrees = targetPitch.getDegrees();
    // This method will be called once per scheduler run
    double clampedError = MathUtil.clamp(getPitch().getDegrees(), targetDegrees - MAX_ERROR, targetDegrees + MAX_ERROR);
    double pidOutput = pitchController.calculate(clampedError, targetDegrees);
    if(!pitchController.atSetpoint()){
      pitchMotor.setVoltage(pidOutput);
    }
  }
}
