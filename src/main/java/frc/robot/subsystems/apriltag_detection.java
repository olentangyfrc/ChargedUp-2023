// Code is based on:
// https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/1

package frc.robot.subsystems;

import frc.robot.SubsystemManager;
import frc.robot.telemetry.OzoneImu;

import java.util.EnumSet;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.Logger;
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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Things to do: Make sure vision works on charge station

public class apriltag_detection extends SubsystemBase {
  String tagFamily = "tag16h5";
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  OzoneImu gyro = SubsystemManager.getInstance().getImu();
  DoubleArrayTopic bot_pose;
  SwerveDrivePoseEstimator poseEstimator = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();
  
  public void init(){
    apriltagVisionThreadProc();

  }

  public void networktables_listener() { 
    NetworkTable LL = inst.getTable("SmartDashboard"); //delcares the networktables to the already intizialized instance
    bot_pose = LL.getDoubleArrayTopic("botpose_wpiblue");
    SmartDashboard.putNumber("bot_pose", NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6])[0]);
    SmartDashboard.putBoolean("LL SEE", false);
    inst.addListener(bot_pose, EnumSet.of(NetworkTableEvent.Kind.kTopic), event -> { try {
      networkTables();
      SmartDashboard.putBoolean("LL SEE", true);
    } catch (JsonProcessingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } } );
   

  }

  public void apriltagVisionThreadProc() {
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily(tagFamily, 0);

    // Microsoft Lifecam Parameters:
    // https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    // Note: These parameters won't work with the tag family I've been testing with.
    // However, they will work with the actual tag family that is being used.
    AprilTagPoseEstimator.Config poseEstConfig =
        new AprilTagPoseEstimator.Config(
            0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
    AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    camera.setExposureManual(50);
    camera.setExposureHoldCurrent();

    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("USB Camera", 640, 480);

    Mat mat = new Mat();
    Mat grayMat = new Mat();

    while (!Thread.interrupted()) {
      if (cvSink.grabFrame(mat) == 0) {
        outputStream.notifyError(cvSink.getError());
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);

      for (AprilTagDetection detection : detections) {
        Transform3d pose = estimator.estimate(detection);
        double lastVisionTime = Timer.getFPGATimestamp();
        Matrix final_vec = transform(pose);


        Matrix apriltag_1 = VecBuilder.fill(Units.inchesToMeters(610.77),Units.inchesToMeters(42.19),Units.inchesToMeters(18.22));
        Matrix apriltag_8 = VecBuilder.fill( Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22));
        Matrix apriltag_7 = VecBuilder.fill( Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22));
        Matrix apriltag_6 = VecBuilder.fill(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22));
        
        Matrix position;
        /* 
        if(detection.getId()==1){
          position =  apriltag_1.minus(final_vec);
          addVision(position, lastVisionTime);
        }else if (detection.getId()==8){
          position =  apriltag_8.minus(final_vec);
          addVision(position, lastVisionTime);
        }else if (detection.getId()==7){
          position =  apriltag_7.minus(final_vec);
          addVision(position, lastVisionTime);
        }else if (detection.getId()==6){
          position =  apriltag_6.minus(final_vec);
          addVision(position, lastVisionTime);
        }
        */
      }

      if (detections.length == 0) {
        System.out.println("Apriltag Not Detected");
      }
      
    }
    detector.close();
  }

  public Matrix transform(Transform3d pose){
    Matrix in_vec = VecBuilder.fill(pose.getX(), pose.getY(), pose.getZ());

    Matrix cv2_correction_mat = Matrix.mat(Nat.N3(), Nat.N3()).fill(
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0);

      Matrix corrected_vec = cv2_correction_mat.times(in_vec);

  

    // Build our rotation matrix
    double pitch = -32.25 * Math.PI / 180; //Change Angle
    double c = Math.cos(pitch);
    double s = Math.sin(pitch);
    Matrix camera_to_bot = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, 0, s,
        0, 1, 0,
      -s, 0, c);

    // Rotate
    Matrix corrected_bot_oriented = camera_to_bot.times(corrected_vec);


    Matrix trans2_vec = VecBuilder.fill(-0.09, -0.1302, -0.3683); //Change this where we know the displacement of the camera to the center of the robot
    
    Matrix out_vec = trans2_vec.plus(corrected_bot_oriented);


    
    double gyro_angle = SubsystemManager.getInstance().getImu().getAngle();
    c = Math.cos(gyro_angle);
    s = Math.sin(gyro_angle);
    Matrix bot_to_field = Matrix.mat(Nat.N3(), Nat.N3()).fill(
        c, -s, 0,
        s, c, 0,
        0, 0, 1);
    Matrix final_vec = bot_to_field.times(out_vec);
    

    SmartDashboard.putNumber("pos_x", final_vec.get(0,0));

    return final_vec;

  }

  public void networkTables() throws JsonMappingException, JsonProcessingException {
    NetworkTable LimelightTable = inst.getTable("limelight");
    double[] LL_pose = LimelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    SmartDashboard.putNumber("test", 1);

    NetworkTableEntry jsonDumpNetworkTableEntry = LimelightTable.getEntry("json");

    String jsonDump = jsonDumpNetworkTableEntry.getString("{}");  
    double tsValue = 0;
    try {
      ObjectMapper mapper = new ObjectMapper();
      JsonNode jsonNodeData = mapper.readTree(jsonDump);
      tsValue = jsonNodeData.path("Results").path("tl").asDouble();
    } catch (JsonProcessingException e) {
      SmartDashboard.putString(jsonDump, jsonDump);
    }
    Rotation2d LL_angle = new Rotation2d(LL_pose[6]);
    addVision(VecBuilder.fill(LL_pose[0], LL_pose[1]), tsValue, LL_angle);
  }

  private void addVision(Matrix position, double lastVisionTime, Rotation2d angle){
    if(position.get(0, 0) > 0 & position.get(1,0) > 0){
      Pose2d robot_pose = (new Pose2d(position.get(0, 0), position.get(1, 0), angle));
      if(poseEstimator.getEstimatedPosition().minus(robot_pose).getX() < 1 & poseEstimator.getEstimatedPosition().minus(robot_pose).getY() < 1){
        if(poseEstimator.getEstimatedPosition().getRotation().minus(angle).getDegrees() < 10){
          poseEstimator.addVisionMeasurement(robot_pose, lastVisionTime);
        }
      }

    }
  
  }

  @Override
  public void periodic(){
    if(inst.getTable("limelight").getEntry("tv").getDouble(0) == 1.0){
      try {
        networkTables();
        SmartDashboard.putBoolean("LL SEE", true);

      } catch (JsonMappingException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      } catch (JsonProcessingException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
  }
}


