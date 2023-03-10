// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.telemetry.commands.autoBalancePitchGroup;
import frc.robot.subsystems.telemetry.commands.resetGyro;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Compressor compressor;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override

  public void robotInit() {
    PathPlannerServer.startServer(5811);

    SubsystemManager.getInstance().init();
    //paths = new AutonPaths(SubsystemManager.getInstance().getDrivetrain());
    SmartDashboard.putData("Auto Balance", new autoBalancePitchGroup(/*new Rotation2d(0)*/));
    SmartDashboard.putData("Reset Gyro", new resetGyro());
    SubsystemManager.getInstance().getDrivetrain().resetLocation(new Pose2d(1.772, 1.149, Rotation2d.fromDegrees(0)));

    // SubsystemManager.getInstance().getDrivetrain().resetLocation(new Pose2d(1.772, 1.149, Rotation2d.fromDegrees(0)));;
    
    // Thread visionThread = new Thread(() -> SubsystemManager.getInstance().getDetector().init());
    // visionThread.setDaemon(true);
    // visionThread.start();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Pitch", SubsystemManager.getInstance().getImu().getPitch());
    SmartDashboard.putNumber("Roll", SubsystemManager.getInstance().getImu().getRoll());
  }
  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {}

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
  public void testInit() {}

  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
