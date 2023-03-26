package frc.robot.auton.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.telemetry.OzoneImu;
import pabeles.concurrency.IntOperatorTask.Max;
import edu.wpi.first.wpilibj.Timer;

public class TaxiRedLeft extends SequentialCommandGroup {
  /** Creates a new autoBalance. */
  public TaxiRedLeft(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        Commands.runOnce(() -> {
          SubsystemManager.getInstance().getImu().resetPitch();
          SubsystemManager.getInstance().getImu().resetRoll();
          SubsystemManager.getInstance().getImu().reset();
        }),
        new MoveLeft(drivetrain),
        new TaxiOutFar(drivetrain)
    );
  }
}