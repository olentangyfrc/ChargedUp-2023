// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auton.AutoRoutineManager;
import frc.robot.auton.AutonPaths;
import frc.robot.auton.AutonPaths.AutoTrajectory;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.drivetrain.commands.DisableBrakeMode;
import frc.robot.telemetry.commands.AutoBalance;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CommandBase autoCommand;
  private AutoRoutineManager routineManager;

  private Compressor compressor;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override

  public void robotInit() {
    PathPlannerServer.startServer(5811);

    SubsystemManager sm = SubsystemManager.getInstance();
    sm.init();
    routineManager = new AutoRoutineManager(sm.getDrivetrain(), sm.getActiveIntake(), sm.getClaw(), sm.getClawPitch(), sm.getElevator());

    // CameraServer.startAutomaticCapture();
    SubsystemManager.getInstance().getDrivetrain().resetLocation(new Pose2d(1.88, 5, Rotation2d.fromDegrees(0)));    
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("Pitch", SubsystemManager.getInstance().getImu().getPitch());
    // SmartDashboard.putNumber("Roll", SubsystemManager.getInstance().getImu().getRoll());
  }
  
  @Override
  public void autonomousInit() {
    SubsystemManager.getInstance().getClaw().setClawPosition(ClawPosition.CLOSED);
    autoCommand = routineManager.getSelectedRoutine();
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    SubsystemManager.getInstance().getDrivetrain().stop();
    if(autoCommand != null) {
      autoCommand.cancel();
    }
    SubsystemManager.getInstance().getActiveIntake().setForceBeamBreak(false);
    SubsystemManager.getInstance().getActiveIntake().setForceBeamOpen(false);
    (new DisableBrakeMode(SubsystemManager.getInstance().getDrivetrain())).schedule();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    SubsystemManager.getInstance().getElevator().zero();
    SubsystemManager.getInstance().getClaw().zero();
    SubsystemManager.getInstance().getClawPitch().zero();

    SubsystemManager.getInstance().getClaw().setClawPosition(ClawPosition.CLOSED);
    SubsystemManager.getInstance().getActiveIntake().retract();
    
  }

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
    SubsystemManager.getInstance().getActiveIntake().retract();
  }
}
