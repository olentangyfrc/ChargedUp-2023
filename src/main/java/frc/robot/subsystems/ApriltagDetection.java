// Code is based on:
// https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/1

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemManager;

//Things to do: Make sure vision works on charge station

public class ApriltagDetection extends SubsystemBase {
  // Angle of april tags to ignore
  private static final double APRIL_TAG_IGNORE_BAND = 30;

  private SwerveDrivePoseEstimator poseEstimator = SubsystemManager.getInstance().getDrivetrain()
      .getSwerveDrivePoseEstimator();
  private PhotonCamera camera = new PhotonCamera("4611_LL3");
  private String path = Filesystem.getDeployDirectory().toPath().resolve("aprilTagFieldLayout.json").toString();
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;
  private Optional<EstimatedRobotPose> poseobject;

  private EstimatedRobotPose robotPose;
  private PhotonPipelineResult result;

  Transform3d robotToCam = new Transform3d(new Translation3d(-0.25, 0.315, 0.33),
      new Rotation3d(-0.0698132, -0.331612558, Math.PI)); // Cam mounted facing forward, half a meter forward of center,
                                                        // half a meter up from center.

                                                        
  public void init() {
    Layout();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera,
        robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  public void Layout() {
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(path);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }

  public void photonvision(PhotonPipelineResult result) throws IOException {
    List<PhotonTrackedTarget> targets = result.targets;
    for (int i = 0; i < targets.size(); i++) {
      double relativeAngle = targets.get(i).getYaw();
      double distance = Math.hypot(targets.get(i).getBestCameraToTarget().getX(), targets.get(i).getBestCameraToTarget().getY());
      SmartDashboard.putNumber("Distance from Tag", distance);
      SmartDashboard.putNumber(targets.get(i).getFiducialId() + " Target Angle:", relativeAngle);
      // Filter for proportion of image size
      if (targets.get(i).getArea() < 0.05) {
        targets.remove(i);
        i--;
      // Filter for ambiguity
      } else if(targets.get(i).getPoseAmbiguity() > 0.2) {
        targets.remove(i);
        i--;
      // Filter out targets we are facing head-on
      } else if(distance < 3.5 && Math.abs(relativeAngle) < APRIL_TAG_IGNORE_BAND / 2) {
        // System.out.println("REMOVE TARGET " + targets.get(i).getFiducialId());
        targets.remove(i);
        i--;

      }
    }
    

    // Construct PhotonPoseEstimator
    poseobject = photonPoseEstimator.update(result);
    // photonPoseEstimator.
      
    try {
      robotPose = poseobject.get();
      // check if its on the ground
      SmartDashboard.putNumber("pose_z", robotPose.estimatedPose.getZ());
      SmartDashboard.putNumber("pose_x", robotPose.estimatedPose.getX());
      SmartDashboard.putNumber("pose_y", robotPose.estimatedPose.getY());
      if (-0.25 < robotPose.estimatedPose.getZ() && robotPose.estimatedPose.getZ() < 0.25) {

        addVision(robotPose.estimatedPose, robotPose.timestampSeconds, result.getBestTarget().getAlternateCameraToTarget().getX());
      }
    } catch (Exception e) {
      // TODO: handle exception
    }

  }

  private void addVision(Pose3d position, double lastVisionTime, double distance) {
    // check it is in the field
    if ((position.getX() > 0) && (position.getY() > 0)) {
      SmartDashboard.putBoolean("Sees Vision", true);
      // poseEstimator.addVisionMeasurement(position.toPose2d(), lastVisionTime, VecBuilder.fill(distance/2, distance/2, 0));

      poseEstimator.addVisionMeasurement(position.toPose2d(), lastVisionTime, VecBuilder.fill(0, 0, 0));
      // gyro.setReset(position.getRotation().toRotation2d());
    }
  }

  @Override
  public void periodic() {
    //photonPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
    result = camera.getLatestResult();
    if (result.hasTargets()) {
      try {
        photonvision(result);
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
}
