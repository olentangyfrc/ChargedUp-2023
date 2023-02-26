// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.activeintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ActiveIntake extends SubsystemBase {
  public static final double UPPER_MOTOR_SPEED = 1;
  public static final double LOWER_MOTOR_SPEED = 1;
  
  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;

  private DoubleSolenoid intakeSolenoid;

  private DigitalInput beamBreaker;

  // TODO: add beam breaks

  /** Creates a new ActiveIntake. */
  public ActiveIntake(int upperMotorCAN, int lowerMotorCAN, int forwardPneumaticChannel, int reversePneumaticChannel) {
    upperMotor = new CANSparkMax(upperMotorCAN, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(lowerMotorCAN, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, forwardPneumaticChannel, reversePneumaticChannel);

    beamBreaker = new DigitalInput(5);

    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setInverted(true);
    lowerMotor.setInverted(false);
  }

  public void setUpperMotor(double speed) {
    if(!(beamBreaker.get())){
      upperMotor.set(speed);
    }
    else{
      upperMotor.stopMotor();
    }
    System.out.println("SET UPPER MOTOR");
  }

  public void setLowerMotor(double speed) {
    if(!(beamBreaker.get())){
      lowerMotor.set(speed);
    }
    else{
      lowerMotor.stopMotor();
    } 
  }

  public void deploy() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }

  public boolean isDeployed() {
    return intakeSolenoid.get() == Value.kForward;
  }
}
