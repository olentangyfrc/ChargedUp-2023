// Code is based on:
// https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/1

package frc.robot.subsystems;

import java.io.IOException;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.SubsystemManager;
import frc.robot.telemetry.OzoneImu;

//Things to do: Make sure vision works on charge station

public class ApriltagDetection extends SubsystemBase {
  private String tagFamily = "tag16h5";
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private OzoneImu gyro = SubsystemManager.getInstance().getImu();
  private DoubleArrayTopic bot_pose;
  private SwerveDrivePoseEstimator poseEstimator = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();
  private PhotonCamera camera = new PhotonCamera("OV5647");

  public void init(){
//     apriltagVisionThreadProc();
  }



  public void photonvision(List<PhotonTrackedTarget> targets) throws IOException{
    var targetArray = targets.toArray();
    int counter = 0;
    //Apritag area bigger than 5%
    for(int i = 0; i < targetArray.length; i++){
      if (((PhotonTrackedTarget) targetArray[i]).getArea() > 5){
        counter++;
      }else{
        //camera.takeOutputSnapshot();
      }
      //Save Images when Ambiguity is less than 5
      SmartDashboard.putNumber("Ambiguity", ((PhotonTrackedTarget) targetArray[i]).getPoseAmbiguity());
      if(((PhotonTrackedTarget) targetArray[i]).getPoseAmbiguity() < 5){
        //camera.takeOutputSnapshot();
      }

    }
    //if all tag area is above 5%
    if (counter == targetArray.length){
      if (((PhotonPipelineResult) targets).getBestTarget().getArea() > 10){
        var aprilTagFieldLayout = new AprilTagFieldLayout("aprilTagFieldLayout.JSON");
        //Cam to Robot
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        
        // Construct PhotonPoseEstimator
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();
        EstimatedRobotPose camPose = result.get();

        //check if its on the ground
        if(-1 < camPose.estimatedPose.getZ() && camPose.estimatedPose.getZ() < 1){
          addVision(camPose.estimatedPose, camPose.timestampSeconds);
        }
      }
    }

  }

  private void addVision(Pose3d position, double lastVisionTime){
    //check it is in the field
    if((position.getX() > 0) && (position.getY() > 0)){
      SmartDashboard.putBoolean("In Field", true);
      poseEstimator.addVisionMeasurement(position.toPose2d(), lastVisionTime, VecBuilder.fill(1, 1, 1));
      gyro.setReset(position.getRotation().toRotation2d());
    }
  }

  @Override
  public void periodic(){
    var result = camera.getLatestResult();
    if(result.hasTargets()){
      try {
        photonvision(result.getTargets());
      } catch (IOException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
}


