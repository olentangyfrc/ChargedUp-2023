// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public static final int MOTOR_TICKS_PER_ROTATION = 2048;
  public static final double GEAR_RATIO = 5; // TODO: Find actual gear ratio

  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCEL = 6;

  public static final double POSITION_TOLERANCE = 0.05;
  private static final double MAX_ERROR = 0.4;

  private static final double MAG_SWITCH_HEIGHT = 2.2;

  private WPI_TalonFX elevatorMotor;

  // private PIDController elevatorController = new PIDController(6, 0, 0); // TODO: Get actual values
  private PIDController elevatorController = new PIDController(17, 0, 0.02);
  // private PIDController elevatorController = new PIDController(5.2, 0, 0.02);

  private Constraints profileConstraints = new Constraints(MAX_VELOCITY, MAX_ACCEL);
  private TrapezoidProfile currentProfile = null;
  private double profileStartTime = 0;

  // private PIDController elevatorController = new PIDController(10.94, 0, 1.0937); // TODO: Get actual values


  // TODO: Add limit switch

  private double targetPosition = 1;

  private boolean isManualControl = false;
  public static final double MANUAL_SPEED = 0.2;
  
  private DoubleSolenoid elevatorSolenoid;

  private DigitalInput magSwitch = new DigitalInput(7);
  
  private static final Map<ElevatorPosition, Double> positionValues = Map.of(
    ElevatorPosition.GROUND, 0.0,
    ElevatorPosition.MIDDLE, 1.5,
    ElevatorPosition.HIGH, 3.0
  );
    
  private GenericEntry entry = Shuffleboard.getTab(getName()).add("Set pos", 0).getEntry();

  private CommandBase moveElevator = Commands.runOnce(() -> setTargetPosition(entry.getDouble(0)));

  /** Creates a new Elevator. */
  public Elevator(int elevatorCanId, int solenoidForward, int solenoidReverse) {
    elevatorMotor = new WPI_TalonFX(elevatorCanId);

    elevatorMotor.configFactoryDefault();


    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorController.setTolerance(POSITION_TOLERANCE);
    
    resetPosition(0);

    elevatorController.setSetpoint(getPosition());

    elevatorSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, solenoidForward, solenoidReverse);

    Shuffleboard.getTab(getName()).addNumber("Elevator position", this::getPosition);
    Shuffleboard.getTab(getName()).addNumber("Elevator target position", this::getTargetPosition);
    Shuffleboard.getTab(getName()).addNumber("Elevator velocity", () -> elevatorMotor.getSelectedSensorVelocity() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO);
    Shuffleboard.getTab(getName()).addBoolean("Mag switch", magSwitch::get);
    Shuffleboard.getTab(getName()).add("Move elevator", moveElevator);
  }

  @Override
  public void periodic() {
    if(getPosition() > 4) {
      elevatorMotor.stopMotor();
      return;
    }
    if(!magSwitch.get()) {
      // elevatorMotor.setSelectedSensorPosition(MAG_SWITCH_HEIGHT * MOTOR_TICKS_PER_ROTATION * GEAR_RATIO);
    }
    if(!isManualControl) {
      if(currentProfile != null) {
        double time = Timer.getFPGATimestamp() - profileStartTime;
        if(currentProfile.isFinished(time)) {
          currentProfile = null;
        } else {
          elevatorController.setSetpoint(currentProfile.calculate(time).position);
        }
      }

      if(!elevatorController.atSetpoint()) {
        double clampedMeasurement = MathUtil.clamp(getPosition(), elevatorController.getSetpoint() - MAX_ERROR, elevatorController.getSetpoint() + MAX_ERROR);
        double pidControl = elevatorController.calculate(clampedMeasurement);
        // System.out.println(pidControl);
        // elevatorMotor.setVoltage(Math.copySign(Math.min(1, pidControl), pidControl));
        // elevatorMotor.setVoltage(elevatorRateLimiter.calculate(pidControl));
        elevatorMotor.setVoltage(pidControl);
        SmartDashboard.putNumber("PPosition Error", (getPosition() - elevatorController.getSetpoint()));
        System.out.println("PID output: " + pidControl);
        // System.out.println("Elevator Error" + elevatorController.getPositionError());
        System.out.println("Setpoint: " + elevatorController.getSetpoint());
        System.out.println("Current Position: " + clampedMeasurement);
      }

    }
  }

  public void resetPosition(double position) {
    elevatorMotor.setSelectedSensorPosition(position * MOTOR_TICKS_PER_ROTATION * GEAR_RATIO);
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

    currentProfile = new TrapezoidProfile(profileConstraints, new State(position, 0), new State(getPosition(), getVelocity()));
    profileStartTime = Timer.getFPGATimestamp();
    elevatorController.setSetpoint(currentProfile.calculate(0).position);
    System.out.println("Elevator.setTargetPosition()");
  }

  public void setTargetPosition(ElevatorPosition position) {
    setTargetPosition(positionValues.get(position));
  }

  public double getPosition() {
    return elevatorMotor.getSelectedSensorPosition() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
  }

  public double getVelocity() {
    return elevatorMotor.getSelectedSensorVelocity() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
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
