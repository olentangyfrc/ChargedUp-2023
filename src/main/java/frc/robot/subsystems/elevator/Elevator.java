// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static final int MOTOR_TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 5; // TODO: Find actual gear ratio

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCEL = 10;

  public static final double POSITION_TOLERANCE = 0.05;
  private static final double MAX_ERROR = 1.5;

  private static final double MAG_SWITCH_HEIGHT = 2.2;

  private WPI_TalonFX elevatorMotor;

  // private PIDController elevatorController = new PIDController(6, 0, 0); // TODO: Get actual values
  private PIDController elevatorController = new PIDController(4, 0, 0.03);
  // private PIDController elevatorController = new PIDController(10.94, 0, 1.0937); // TODO: Get actual values
  private ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.32249, 0.20783, 0.56769, 0.040178);

  // TODO: Add limit switch

  private double targetPosition = 1;

  private boolean isManualControl = false;
  public static final double MANUAL_SPEED = 0.1;

  private DoubleSolenoid elevatorSolenoid;

  private DigitalInput magSwitch = new DigitalInput(7);

  private static final Map<ElevatorPosition, Double> positionValues = Map.of(
    ElevatorPosition.GROUND, 0.0,
    ElevatorPosition.MIDDLE, 1.5,
    ElevatorPosition.HIGH, 3.0
  );

  /** Creates a new Elevator. */
  public Elevator(int elevatorCanId, int solenoidForward, int solenoidReverse) {
    elevatorMotor = new WPI_TalonFX(elevatorCanId);

    elevatorMotor.configFactoryDefault();
    elevatorMotor.setSelectedSensorPosition(3 * MOTOR_TICKS_PER_ROTATION * GEAR_RATIO);


    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorController.setTolerance(POSITION_TOLERANCE);
    elevatorController.setSetpoint(4);

    elevatorSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, solenoidForward, solenoidReverse);

    Shuffleboard.getTab(getName()).addNumber("Elevator position", this::getPosition);
    Shuffleboard.getTab(getName()).addNumber("Elevator target position", this::getTargetPosition);
    Shuffleboard.getTab(getName()).addNumber("Elevator velocity", () -> elevatorMotor.getSelectedSensorVelocity() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO);
    Shuffleboard.getTab(getName()).addBoolean("Mag switch", magSwitch::get);
  }

  private GenericEntry entry = Shuffleboard.getTab(getName()).add("Set pos", 4).getEntry();

  @Override
  public void periodic() {
    if(getPosition() < 2) {
      elevatorMotor.stopMotor();
      return;
    }
    if(!magSwitch.get()) {
      // elevatorMotor.setSelectedSensorPosition(MAG_SWITCH_HEIGHT * MOTOR_TICKS_PER_ROTATION * GEAR_RATIO);
    }
    if(!isManualControl) {
      setTargetPosition(entry.getDouble(4));
      if(!elevatorController.atSetpoint()) {
        double clampedMeasurement = MathUtil.clamp(getPosition(), elevatorController.getSetpoint() - MAX_ERROR, elevatorController.getSetpoint() + MAX_ERROR);
        double pidControl = elevatorController.calculate(clampedMeasurement, targetPosition);
        // System.out.println(pidControl);
        // elevatorMotor.setVoltage(Math.copySign(Math.min(1, pidControl), pidControl));
        // elevatorMotor.setVoltage(pidControl);
        System.out.println("PID output: " + pidControl);
        // System.out.println("Elevator Error" + elevatorController.getPositionError());
        System.out.println("Setpoint: " + elevatorController.getSetpoint());
        System.out.println("Current Position: " + clampedMeasurement);
      } else {
        elevatorMotor.setVoltage(0);
      }
    }
  }

  public void moveElevatorForward() {
    elevatorSolenoid.set(Value.kForward);
  }

  public void moveElevatorBack() {
    elevatorSolenoid.set(Value.kReverse);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isManualControl() {
    return isManualControl;
  }

  public void setManualControl(boolean isManualControl) {
    this.isManualControl = isManualControl;
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
    elevatorController.setSetpoint(position);;
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
