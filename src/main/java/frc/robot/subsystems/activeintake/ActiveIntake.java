// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.activeintake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private static final double GRAB_WAIT_TIME = 1;
  private boolean isClawHoldingGamePiece = false;
  private boolean isWaiting = false;
  private boolean intakeHasRun = false;

  private boolean nextPieceIsCone = true;
  private boolean forceBelts = false;

  private CommandBase grabCommand;


  private double startTimer = Timer.getFPGATimestamp();

  private Claw claw;

  /** Creates a new ActiveIntake. */
  public ActiveIntake(int upperMotorCAN, int lowerMotorCAN, int forwardPneumaticChannel, int reversePneumaticChannel) {
    upperMotor = new CANSparkMax(upperMotorCAN, MotorType.kBrushless);
    lowerMotor = new CANSparkMax(lowerMotorCAN, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, forwardPneumaticChannel, reversePneumaticChannel);

    beamBreaker = new DigitalInput(8);

    upperMotor.restoreFactoryDefaults();
    lowerMotor.restoreFactoryDefaults();

    upperMotor.setInverted(false);
    lowerMotor.setInverted(false);

    Shuffleboard.getTab(getName()).addBoolean("Beam Break", this::isBeamBroken);
  }
  GenericEntry isConeEntry = Shuffleboard.getTab(getName()).add("Next piece cone", true).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  public boolean isClawHoldingGamePiece() {
    return isClawHoldingGamePiece;
  }

  public void setClawHoldingGamePiece(boolean isClawHoldingGamePiece) {
    if(isClawHoldingGamePiece) {
      intakeHasRun = false;
    }
    this.isClawHoldingGamePiece = isClawHoldingGamePiece;
  }

  @Override
  public void periodic(){
    nextPieceIsCone = isConeEntry.getBoolean(true);
    if(!isBeamBroken() && grabCommand != null) {
      grabCommand.cancel();
      grabCommand = null;
      SubsystemManager.getInstance().getElevator().stopElevator();
    }
    if(isBeamBroken() && !isClawHoldingGamePiece) {
      if(!isWaiting) {
        startTimer = Timer.getFPGATimestamp();
        isWaiting = true;
      } else {
        if(Timer.getFPGATimestamp() - startTimer >= (nextPieceIsCone? GRAB_WAIT_TIME : 0)) {
          if(grabCommand == null) {
            grabCommand = new GrabGamePiece(
              SubsystemManager.getInstance().getClaw(),
              SubsystemManager.getInstance().getClawPitch(),
              SubsystemManager.getInstance().getElevator(),
              nextPieceIsCone
            );
  
            grabCommand.andThen(new InstantCommand(() -> {
              grabCommand = null;
              isClawHoldingGamePiece = true;
              isWaiting = false;
              intakeHasRun = false;
            })).schedule();
          }
            
        }
      }
    } else {
      isWaiting = false;
    }

    if(!forceBelts) {
      if(!isClawHoldingGamePiece && intakeHasRun) {
        lowerMotor.set(LOWER_MOTOR_SPEED);
      } else {
        lowerMotor.stopMotor();
      }
    }
  }

  public void setForceBelts(boolean forceBelts) {
    this.forceBelts = forceBelts;
  }

  public void setUpperMotor(double speed) {
    if(speed > 0) {
      intakeHasRun = true;
    }
    upperMotor.set(speed);
    // System.out.println("SET UPPER MOTOR");
  }

  public boolean nextPieceIsCone() {
    return nextPieceIsCone;
  }

  public void setLowerMotor(double speed) {
    lowerMotor.set(speed);
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
