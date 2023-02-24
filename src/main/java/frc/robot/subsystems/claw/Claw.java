// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  // Wrist
  private static final double MAX_WRIST_ERROR = 0.1;
  private static final double WRIST_ANGLE_TOLERANCE = 0.05;
  private CANSparkMax wristMotor;
  private PIDController wristController = new PIDController(1, 0, 0); // TODO: Find the actual values!!!
  // TODO: Determine how to get the wrist's angle

  private Rotation2d targetWristAngle = new Rotation2d();

  private DoubleSolenoid upperSolenoid;
  private DoubleSolenoid lowerSolenoid;

  /** Creates a new Claw. */
  public Claw(int wristCanId, int upperForwardChannel, int upperReverseChannel, int lowerForwardChannel, int lowerReverseChannel) {
    wristMotor = new CANSparkMax(wristCanId, MotorType.kBrushless);

    upperSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, upperForwardChannel, upperReverseChannel);
    lowerSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, lowerForwardChannel, lowerReverseChannel);

    wristController.setTolerance(WRIST_ANGLE_TOLERANCE);
  }

  @Override
  public void periodic() {
    double targetRadians = targetWristAngle.getRadians();
    double clampedCurrentAngle = MathUtil.clamp(getWristAngle().getRadians(), targetRadians - MAX_WRIST_ERROR, targetRadians + MAX_WRIST_ERROR);
    wristMotor.setVoltage(wristController.calculate(clampedCurrentAngle));
  }

  public void setTargetWristAngle(Rotation2d angle) {
    targetWristAngle = new Rotation2d(angle.getRadians() % (2 * Math.PI));
  }

  public void setClawPosition(ClawPosition position) {
    switch(position) {
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

  // TODO
  /**
   * Get the wrist's current angle on the interval [0, 2pi) radians
   * 
   * @return
   */
  public Rotation2d getWristAngle() {
    return new Rotation2d();
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
