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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;

public class Claw extends SubsystemBase {
  // Wrist
  private static final double WRIST_GEAR_RATIO = 90;
  private static final double MAX_WRIST_ERROR = 0.7854;
  private static final double WRIST_ANGLE_TOLERANCE = 0.0698132;

  public static final double MANUAL_SPEED = 2;

  private CANSparkMax wristMotor;

  // private PIDController wristController = new PIDController(0.5, 0, 0);
  private PIDController wristController = new PIDController(6, 0, 0.8);

  private Rotation2d targetWristAngle = new Rotation2d(Math.PI);

  private DoubleSolenoid upperSolenoid;
  private DoubleSolenoid lowerSolenoid;

  private boolean isManualControl = false;

  private ClawPosition clawPosition = ClawPosition.CLOSED;

  /** Creates a new Claw. */
  public Claw(int wristCanId, int upperForwardChannel, int upperReverseChannel, int lowerForwardChannel,
      int lowerReverseChannel) {
    wristMotor = new CANSparkMax(wristCanId, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.setInverted(true);
    wristMotor.setIdleMode(IdleMode.kBrake);

    upperSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, upperForwardChannel, upperReverseChannel);
    lowerSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, lowerForwardChannel, lowerReverseChannel);

    wristController.setTolerance(WRIST_ANGLE_TOLERANCE);

    Shuffleboard.getTab(getName()).addNumber("Claw position", () -> getWristAngle().getDegrees());
    Shuffleboard.getTab(getName()).addNumber("Current Radians", () -> getWristAngle().getRadians());
    Shuffleboard.getTab(getName()).addNumber("Target Radians", () -> targetWristAngle.getRadians());
    Shuffleboard.getTab(getName()).add("Forwards", new RotateClawToAngle(this, Rotation2d.fromDegrees(0)));
    Shuffleboard.getTab(getName()).add("Reverse", new RotateClawToAngle(this, Rotation2d.fromDegrees(180)));
    Shuffleboard.getTab("Command Groups").addBoolean("Claw At Setpoint", () -> isWristAtAngle());
  }

  @Override
  public void periodic() {
    if(!isManualControl) {
      double targetRadians = targetWristAngle.getRadians();
      double clampedCurrentAngle = MathUtil.clamp(getWristAngle().getRadians(), targetRadians - MAX_WRIST_ERROR,
          targetRadians + MAX_WRIST_ERROR);
      SmartDashboard.putNumber("Clamped angle", clampedCurrentAngle);
      double pidOutput = wristController.calculate(clampedCurrentAngle, targetRadians);
  
      if (!wristController.atSetpoint()) {
        wristMotor.setVoltage(pidOutput);
      }
    }
  }

  public void stopWristMotor() {
    wristMotor.stopMotor();
  }

  public boolean isWristAtSetpoint() {
    return wristController.atSetpoint();
  }

  public ClawPosition getClawPosition() {
    return clawPosition;
  }

  public void setTargetClawAngle(Rotation2d targetAngle) {
    targetWristAngle = targetAngle;
    wristController.setSetpoint(targetAngle.getRadians());
  }

  public void setClawPosition(ClawPosition position) {
    clawPosition = position;
    switch (position) {
      case OPEN:
        upperSolenoid.set(Value.kReverse);
        lowerSolenoid.set(Value.kReverse);
        break;
      case CLOSED:
        upperSolenoid.set(Value.kForward);
        lowerSolenoid.set(Value.kForward);
        break;
      case UPPER_LATCH:
        upperSolenoid.set(Value.kForward);
        lowerSolenoid.set(Value.kReverse);
        break;
      case LOWER_LATCH:
        upperSolenoid.set(Value.kReverse);
        lowerSolenoid.set(Value.kForward);
        break;
      case LOOSE:
        upperSolenoid.set(Value.kOff);
        lowerSolenoid.set(Value.kOff);
        break;
    }
  }

  public boolean isClosed() {
    return clawPosition == ClawPosition.CLOSED;
  }

  public void setIsManualControl(boolean isManualControl) {
    this.isManualControl = isManualControl;
  }

  public void zero() {
    wristMotor.getEncoder().setPosition((180.0 / 360) * WRIST_GEAR_RATIO);
    setTargetClawAngle(Rotation2d.fromDegrees(180));
  }

  public void setClawVoltage(double voltage) {
    if(isManualControl) {
      wristMotor.setVoltage(voltage);
    }
  }

  /**
   * Get the wrist's current angle on the interval [0, 2pi) radians
   * 
   * @return
   */
  public Rotation2d getWristAngle() {
    return Rotation2d.fromRotations(wristMotor.getEncoder().getPosition() / WRIST_GEAR_RATIO);
  }

  public boolean isWristAtAngle() {
    return wristController.atSetpoint();
  }

  public static enum ClawPosition {
    OPEN,
    CLOSED,
    UPPER_LATCH,
    LOWER_LATCH,
    LOOSE
  }
}
