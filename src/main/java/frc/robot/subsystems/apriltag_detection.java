// Code is based on:
// https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/1

package frc.robot.subsystems;

import frc.robot.SubsystemManager;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


public class apriltag_detection extends SubsystemBase {
  //String tagFamily = "tag36hll";
  String tagFamily = "tag16h5";
  private SwerveDrivetrain drivetrain;



  void apriltagVisionThreadProc() {
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
        
        var in_vec = VecBuilder.fill(pose.getX(), pose.getY(), pose.getZ());

        var cv2_correction_mat = Matrix.mat(Nat.N3(), Nat.N3()).fill(
          0, 0, 1,
          -1, 0, 0,
          0, -1, 0);

        var corrected_vec = cv2_correction_mat.times(in_vec);

        


        // Build our rotation matrix
        double pitch = -30 * Math.PI / 180; //Change Angle
        double c = Math.cos(pitch);
        double s = Math.sin(pitch);
        var camera_to_bot = Matrix.mat(Nat.N3(), Nat.N3()).fill(
            c, 0, s,
            0, 1, 0,
          -s, 0, c);

        // Rotate
        var corrected_bot_oriented = camera_to_bot.times(corrected_vec);


        var trans2_vec = VecBuilder.fill(-0.1016, -0.381, -0.127); //Change this where we know the displacement of the camera to the center of the robot
        
        var out_vec = trans2_vec.plus(corrected_bot_oriented);


        
        //double gyro_angle = getPastPose(elapsedtime).getEstimatedPosition().getRotation().getRadians();
        double gyro_angle = 180;
        c = Math.cos(gyro_angle);
        s = Math.sin(gyro_angle);
        var bot_to_field = Matrix.mat(Nat.N3(), Nat.N3()).fill(
            c, -s, 0,
            s, c, 0,
            0, 0, 1);
        var final_vec = bot_to_field.times(out_vec);
        
        //var apriltag_1 =  new Pose3d(Units.inchesToMeters(610.77),Units.inchesToMeters(42.19),Units.inchesToMeters(18.22),new Rotation3d(0.0, 0.0, 0.0))
        var apriltag_1 = VecBuilder.fill(Units.inchesToMeters(610.77),Units.inchesToMeters(42.19),Units.inchesToMeters(18.22));
        

        if(detection.getId()=='1'){
          var position =  apriltag_1.minus(final_vec);
          var robot_pose = new Pose2d(position.get(0, 0), position.get(1, 0), new Rotation2d());
          var poseEstimator = SubsystemManager.getInstance().getDrivetrain().getSwerveDrivePoseEstimator();
          poseEstimator.addVisionMeasurement(robot_pose, lastVisionTime);
          SmartDashboard.putNumber("robot_position_x", poseEstimator.getEstimatedPosition().getX());
          SmartDashboard.putNumber("robot_position_x", poseEstimator.getEstimatedPosition().getX());

        }

      }

      if (detections.length == 0) {
        System.out.println("Apriltag Not Detected");
        
        //System.out.println("**********************");
      }
      outputStream.putFrame(mat);
    }
    detector.close();
  }
}