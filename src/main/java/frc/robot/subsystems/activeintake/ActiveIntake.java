// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.activeintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActiveIntake extends SubsystemBase {
  public static final double UPPER_MOTOR_SPEED = 1;
  public static final double LOWER_MOTOR_SPEED = 1;
  
  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;

  private boolean isUpperMotorOn = false;
  private boolean isLowerMotorOn = false;

  /** Creates a new ActiveIntake. */
  public ActiveIntake(int upperMotorCAN, int lowerMotorCAN) {
    upperMotor = new CANSparkMax(upperMotorCAN, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(lowerMotorCAN, MotorType.kBrushless);

    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setInverted(true);
    lowerMotor.setInverted(true);
  }

  public void setUpperMotor(double speed) {
    upperMotor.set(speed);
  }

  public void setLowerMotor(double speed) {
    lowerMotor.set(speed);
  }

  public boolean isUpperMotorOn() {
    return isUpperMotorOn;
  }

  public boolean isLowerMotorOn() {
    return isLowerMotorOn;
  }
}
