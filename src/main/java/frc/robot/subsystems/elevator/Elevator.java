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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // CONSTANT DECLARATIONS
  private static final int MOTOR_TICKS_PER_ROTATION = 2048;
  private static final double GEAR_RATIO = 5;

  // These are in rps and rps^2 respectively
  public static final double MAX_VELOCITY = 10;
  public static final double MAX_ACCEL = 6;

  private static final double CORRECTION_MAX_VOLTS = 3; // Max voltage for small corrections in position when there isn't a motion profile.

  // These are in elevator motor rotations.
  public static final double POSITION_TOLERANCE = 0.05;
  private static final double MAX_ERROR = 0.4;

  private static final double MAG_SWITCH_HEIGHT = 2.2;

  // DEVICES
  private WPI_TalonFX elevatorMotor;
  private DoubleSolenoid elevatorSolenoid;
  private DigitalInput magSwitch = new DigitalInput(7);

  // CONTROL LOGIC
  // private PIDController elevatorController = new PIDController(6, 0, 0); // TODO: Get actual values
  private PIDController elevatorController = new PIDController(17, 0, 0.02);
  private double goalPosition = 0;
  // private PIDController elevatorController = new PIDController(5.2, 0, 0.02);

  private Constraints profileConstraints = new Constraints(MAX_VELOCITY, MAX_ACCEL);
  private TrapezoidProfile currentProfile = null;

  private double profileStartTime = 0;
  
  private boolean isManualControl = false;
  public static final double MANUAL_SPEED = 0.2;


  private static final Map<ElevatorPosition, Double> positionValues = Map.of(
    ElevatorPosition.GROUND, 0.0,
    ElevatorPosition.GRAB_CONE, 0.86,
    ElevatorPosition.GRAB_CUBE, 1.0,
    ElevatorPosition.LOW, 2.155078125,
    ElevatorPosition.MIDDLE, 5.1099609375,
    ElevatorPosition.HIGH, 5.66328125
  );
  
  // These are only for development purposes
  private GenericEntry entry = Shuffleboard.getTab(getName()).add("Set pos", 0).getEntry();

  private CommandBase moveElevator = Commands.runOnce(() -> goToPosition(entry.getDouble(0)));

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
    Shuffleboard.getTab(getName()).addNumber("Elevator target position", elevatorController::getSetpoint);
    Shuffleboard.getTab(getName()).addNumber("Elevator velocity", () -> elevatorMotor.getSelectedSensorVelocity() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO);
    Shuffleboard.getTab(getName()).addBoolean("Mag switch", magSwitch::get);
    Shuffleboard.getTab(getName()).add("Move elevator", moveElevator);
    Shuffleboard.getTab(getName()).add("Zero Elevator", Commands.runOnce(() -> {
      resetPosition(0);
      setElevatorSetpoint(0);
    }));
  }

  @Override
  public void periodic() {
    // TODO: REMEMBER TO TAKE THIS OUT!!!!!
    // Re-zero our position when we pass the magnetic switch
    if(!magSwitch.get()) {
      // elevatorMotor.setSelectedSensorPosition(MAG_SWITCH_HEIGHT * MOTOR_TICKS_PER_ROTATION * GEAR_RATIO);
    }
    if(!isManualControl) {
      // If we have a profile, update our target position.
      if(currentProfile != null) {
        double time = Timer.getFPGATimestamp() - profileStartTime;
        if(currentProfile.isFinished(time)) {
          currentProfile = null;
        } else {
          elevatorController.setSetpoint(currentProfile.calculate(time).position);
        }
      }

      if(!isAtTargetPosition()) {
        double clampedMeasurement = MathUtil.clamp(getPosition(), elevatorController.getSetpoint() - MAX_ERROR, elevatorController.getSetpoint() + MAX_ERROR);
        double pidControl = elevatorController.calculate(clampedMeasurement);

        // Limit the output voltage when there isn't a motion profile.
        if(currentProfile == null) {
          pidControl = MathUtil.clamp(pidControl, -CORRECTION_MAX_VOLTS, CORRECTION_MAX_VOLTS);
        }

        elevatorMotor.setVoltage(pidControl);

        SmartDashboard.putNumber("PPosition Error", (getPosition() - elevatorController.getSetpoint()));
        // System.out.println("PID output: " + pidControl);
        // System.out.println("Setpoint: " + elevatorController.getSetpoint());
        // System.out.println("Current Position: " + clampedMeasurement);
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

  public void stopElevator() {
    currentProfile = null;
    elevatorMotor.stopMotor();
    elevatorController.setSetpoint(getPosition());
  }

  /**
   * Set the target position of the elevator. This will not run if the robot is disabled.
   * 
   * @param position The position to go to
   * @return True if the position is set, false if the position is not set (This will only happen if the robot is disabled). 
   */
  public boolean goToPosition(double position) {
    if(DriverStation.isDisabled()) {
      return false;
    }

    currentProfile = new TrapezoidProfile(profileConstraints, new State(position, 0), new State(getPosition(), getVelocity()));
    goalPosition = position;
    profileStartTime = Timer.getFPGATimestamp();
    elevatorController.setSetpoint(currentProfile.calculate(0).position);
    System.out.println("Elevator.setTargetPosition()");
    return true;
  }

  public void goToPosition(ElevatorPosition position) {
    goToPosition(positionValues.get(position));
  }

  /**
   * THIS SHOULD NOT BE USED TO GO TO A POSITION!
   * 
   * @param setpoint
   */
  public void setElevatorSetpoint(double setpoint) {
    elevatorController.setSetpoint(setpoint);
  }

  public double getPosition() {
    return elevatorMotor.getSelectedSensorPosition() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
  }

  public double getGoalPosition() {
    if(currentProfile == null) {
      return Double.NaN;
    } else {
      return goalPosition;
    }
  }

  public double getVelocity() {
    return elevatorMotor.getSelectedSensorVelocity() / MOTOR_TICKS_PER_ROTATION / GEAR_RATIO;
  }

  public boolean isAtTargetPosition() {
    return Math.abs(elevatorController.getSetpoint() - getPosition()) <= POSITION_TOLERANCE;
  }

  public static enum ElevatorPosition {
    GROUND,
    GRAB_CONE,
    GRAB_CUBE,
    LOW,
    MIDDLE,
    HIGH
  }
}
