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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemManager;

//Things to do: Make sure vision works on charge station

public class ApriltagDetection extends SubsystemBase {
  private SwerveDrivePoseEstimator poseEstimator = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();
  private PhotonCamera camera = new PhotonCamera("OV5647");
  private String path = Filesystem.getDeployDirectory().toPath().resolve("aprilTagFieldLayout.json").toString();
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonTrackedTarget[] targetArray;
  private PhotonPoseEstimator photonPoseEstimator;
  private Optional<EstimatedRobotPose>  poseobject;
  private EstimatedRobotPose robotPose;

  Transform3d robotToCam = new Transform3d(new Translation3d(-0.25, 0.315, 0.33), new Rotation3d(0,67.38,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.


  public void init(){
    Layout();
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);

  }

  public void Layout(){
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(path);
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }



  public void photonvision(List<PhotonTrackedTarget> targets, PhotonPipelineResult result) throws IOException{
    targetArray = targets.toArray(PhotonTrackedTarget[]::new);
    int counter = 0;
    //Apritag area bigger than 5%
    for(int i = 0; i < targetArray.length; i++){
      if (((PhotonTrackedTarget) targetArray[i]).getArea() > 0.01){
        counter++;
      }

    }
    //if all tag area is above 5%
    if (counter == targetArray.length){
      SmartDashboard.putBoolean("Step 2", true);
      if (result.getBestTarget().getArea() > 0.14){
        SmartDashboard.putBoolean("Step 3", true);
        //Cam to Robot
        
        // Construct PhotonPoseEstimator
        poseobject = photonPoseEstimator.update();
        
        try {
          robotPose = poseobject.get();
                  //check if its on the ground
          SmartDashboard.putNumber("pose_z", robotPose.estimatedPose.getZ());
          if(-1 < robotPose.estimatedPose.getZ() && robotPose.estimatedPose.getZ() < 1){
            SmartDashboard.putBoolean("Step 4", true);
            addVision(robotPose.estimatedPose, robotPose.timestampSeconds);
          }
        } catch (Exception e) {
          // TODO: handle exception
        }
        


      }
    }

  }

  private void addVision(Pose3d position, double lastVisionTime){
    //check it is in the field
    if((position.getX() > 0) && (position.getY() > 0)){
      SmartDashboard.putBoolean("In Field", true);
      poseEstimator.addVisionMeasurement(position.toPose2d(), lastVisionTime, VecBuilder.fill(1, 1, 1));
      //gyro.setReset(position.getRotation().toRotation2d());
    }
  }

  @Override
  public void periodic(){
    var result = camera.getLatestResult();
    if(result.hasTargets()){
      try {
        SmartDashboard.putBoolean("Step 1", true);
        photonvision(result.getTargets(), result);
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
}


