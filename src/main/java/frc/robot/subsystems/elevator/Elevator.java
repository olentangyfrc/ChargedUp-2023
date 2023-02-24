// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static final int MOTOR_TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 1; // TODO: Find actual gear ratio

  public static final double POSITION_TOLERANCE = 0.05;
  private static final double MAX_ERROR = 0.3;

  private WPI_TalonFX elevatorMotor;

  private PIDController elevatorController = new PIDController(19.128, 0, 3.2599); // TODO: Get actual values

  // TODO: Add limit switch

  private double targetPosition = 0;

  private static final Map<ElevatorPosition, Double> positionValues = Map.of(
    ElevatorPosition.GROUND, 0.0,
    ElevatorPosition.MIDDLE, 1.5,
    ElevatorPosition.HIGH, 3.0
  );

  /** Creates a new Elevator. */
  public Elevator(int elevatorCanId) {
    elevatorMotor = new WPI_TalonFX(elevatorCanId);

    elevatorMotor.configFactoryDefault();

    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorController.setTolerance(POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    double clampedMeasurement = MathUtil.clamp(getPosition(), targetPosition - MAX_ERROR, targetPosition + MAX_ERROR);
    double pidControl = elevatorController.calculate(clampedMeasurement, targetPosition);
    elevatorMotor.setVoltage(pidControl);
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public void setTargetPosition(ElevatorPosition position) {
    setTargetPosition(positionValues.get(position));
  }

  public double getPosition() {
    return elevatorMotor.getSelectedSensorPosition() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public boolean isAtTargetPosition() {
    return elevatorController.atSetpoint();
  }

  public static enum ElevatorPosition {
    GROUND,
    MIDDLE,
    HIGH
  }
}
