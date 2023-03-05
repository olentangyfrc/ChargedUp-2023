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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.commands.GrabGamePiece;

public class ActiveIntake extends SubsystemBase {
  public static final double UPPER_MOTOR_SPEED = 1;
  public static final double LOWER_MOTOR_SPEED = 1;
  
  private CANSparkMax upperMotor;
  private CANSparkMax lowerMotor;

  private DoubleSolenoid intakeSolenoid;

  private DigitalInput beamBreaker;

  private boolean isClawHoldingGamePiece;
  private boolean isGamePieceInActiveIntake;

  private double startTimer;

  private Claw claw;

  /** Creates a new ActiveIntake. */
  public ActiveIntake(int upperMotorCAN, int lowerMotorCAN, int forwardPneumaticChannel, int reversePneumaticChannel) {
    upperMotor = new CANSparkMax(upperMotorCAN, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(lowerMotorCAN, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, forwardPneumaticChannel, reversePneumaticChannel);

    beamBreaker = new DigitalInput(6);

    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setInverted(true);
    lowerMotor.setInverted(false);

    isGamePieceInActiveIntake = false;
  }

  @Override
  public void periodic(){
    /*if(isBeamBroken()) {
      startTimer = Timer.getFPGATimestamp();
      if(isBeamBroken() && Timer.getFPGATimestamp() - startTimer >= 1000) {
        isGamePieceInActiveIntake = true;
        new GrabGamePiece(SubsystemManager.getInstance().getClaw());
        isClawHoldingGamePiece = true;
      }
    }
    else {
      setLowerMotor(LOWER_MOTOR_SPEED);
    }
    */
  }

  public void setUpperMotor(double speed) {
    upperMotor.set(speed);
    System.out.println("SET UPPER MOTOR");
  }

  public void setLowerMotor(double speed) {
    lowerMotor.set(speed);
    System.out.println("SET LOWER MOTOR");
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

  public boolean isBeamBroken(){
    return !(beamBreaker.get());
  }
}
