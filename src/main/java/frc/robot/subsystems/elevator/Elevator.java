// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.commands.MoveElevator;

public class Elevator extends SubsystemBase {
  public static final int MOTOR_TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 25;

  private WPI_TalonFX leftElevator = new WPI_TalonFX(10);
  private WPI_TalonFX rightElevator = new WPI_TalonFX(11);

  private MotorControllerGroup elevatorMotors;
  private PIDController elevatorPid = new PIDController(19.128, 0, 3.2599);
  private DigitalInput elevatorLimitSwitch = new DigitalInput(0);

  private double targetPosition = 0;
  public static final double POSITION_TOLERANCE = 0.01;
  private static final double MAX_ERROR = 0.3;

  // Elevator position for scoring (unit is rotations)
  public static final double ELEVATOR_HIGH_POS = 3.8;

  /** Creates a new Elevator. */
  public Elevator() {
  
    rightElevator.configFactoryDefault();
    leftElevator.configFactoryDefault();

    rightElevator.setNeutralMode(NeutralMode.Brake);
    leftElevator.setNeutralMode(NeutralMode.Brake);
    
    rightElevator.setInverted(true);

    elevatorMotors = new MotorControllerGroup(leftElevator, rightElevator);

    elevatorPid.setTolerance(POSITION_TOLERANCE);
    Shuffleboard.getTab("Elevator").addNumber("Elevator Position", this::getElevatorPosition);
    Shuffleboard.getTab("Elevator").addBoolean("Limit Switch", this::getLimitSwitch);
    Shuffleboard.getTab("Elevator").add(new MoveElevator(this, 0));
    Shuffleboard.getTab("Elevator").add(elevatorPid);
  }

  @Override
  public void periodic() {
    if(getLimitSwitch()) {
      leftElevator.setSelectedSensorPosition(0);
    }

    double clampedMeasurement = MathUtil.clamp(getElevatorPosition(), targetPosition - MAX_ERROR, targetPosition + MAX_ERROR);
    double pidControl = elevatorPid.calculate(clampedMeasurement, targetPosition);
    elevatorMotors.setVoltage(pidControl);
  }

  public double getElevatorPosition() {
    return leftElevator.getSelectedSensorPosition() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
  }

  public boolean isAtTargetPosition() {
    return elevatorPid.atSetpoint();
  }

  public boolean getLimitSwitch() {
    return !elevatorLimitSwitch.get();
  }

  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }



  public enum ElevatorLocation {
    GROUND,
    MIDDLE,
    HIGH
  }
}
