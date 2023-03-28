
package frc.robot.subsystems;

import frc.robot.SubsystemManager;
import frc.robot.telemetry.OzoneImu;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private String LL3 = "limelight";
  private boolean enable = false;
  private boolean blink = false;
  private double distanceError = 0;
  private double angleError = 0;
  private Pose2d botpose;
  private SwerveDrivePoseEstimator drivetrain = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();
  private OzoneImu imu = SubsystemManager.getInstance().getImu();
  /** Creates a new Limelight. */
  public Limelight() {
    SmartDashboard.putNumber("Limelight Distance Error", distanceError);
    SmartDashboard.putNumber("Limelight Angle Error", angleError);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Lightlight Step 1", false);
    if (enable) {
      SmartDashboard.putBoolean("Lightlight Step 1", true);
      LimelightHelpers.Results result =
          LimelightHelpers.getLatestResults(LL3).targetingResults;
      if (!(result.botpose[0] == 0 && result.botpose[1] == 0)) {
        SmartDashboard.putBoolean("Lightlight Step 2", true);
        botpose = LimelightHelpers.toPose2D(result.botpose_wpiblue);
        if ((botpose.getX() > 0) && (botpose.getY() > 0)) {
            SmartDashboard.putBoolean("Lightlight Step 3", true);
          if ((drivetrain.getEstimatedPosition().getTranslation().getDistance(botpose.getTranslation()) < 0.33 
              || Math.abs(imu.getAngle()-botpose.getRotation().getDegrees()) < 8)
              || result.targets_Fiducials.length > 1) {
                SmartDashboard.putBoolean("Lightlight Step 4", true);
                drivetrain.addVisionMeasurement(
                botpose,
                Timer.getFPGATimestamp()
                    - (result.latency_capture / 1000.0)
                    - (result.latency_pipeline / 1000.0),
                VecBuilder.fill(0, 0, 0));
          } else {
            distanceError = drivetrain.getEstimatedPosition().getTranslation().getDistance(botpose.getTranslation());
            angleError = Math.abs(imu.getAngle()-botpose.getRotation().getDegrees());
            SmartDashboard.putBoolean("Lightlight Step 4", false);
        }
        } else {
            SmartDashboard.putBoolean("Lightlight Step 3", false);
        }
      }
      SmartDashboard.putBoolean("Lightlight Step 2", false);
    }
  }

  public void setLLBlink(boolean blink) {
    if(blink){
        LimelightHelpers.setLEDMode_ForceBlink(LL3);
    }else{
        LimelightHelpers.setLEDMode_ForceOff(LL3);
    }
  }

  public void useLimelight(boolean enable) {
    this.enable = enable;
  }

}