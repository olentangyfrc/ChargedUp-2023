package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Map;
import java.util.logging.Logger;

import javax.swing.ListModel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.IO.ButtonActionType;
import frc.robot.IO.ControllerButton;
import frc.robot.auton.AutoDashboardManager;
import frc.robot.auton.AutonPaths;
import frc.robot.subsystems.ApriltagDetection;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.DeployIntake;
import frc.robot.subsystems.activeintake.commands.RetractIntake;
import frc.robot.subsystems.activeintake.commands.ReverseIntake;
import frc.robot.subsystems.activeintake.commands.StartIntake;
import frc.robot.subsystems.activeintake.commands.StopIntake;
import frc.robot.subsystems.activeintake.commands.ToggleIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.claw.commands.ManualClawBackwards;
import frc.robot.subsystems.claw.commands.ManualClawForwards;
import frc.robot.subsystems.claw.commands.RotateClawPitch;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;
import frc.robot.subsystems.claw.commands.SetClawPosition;
import frc.robot.subsystems.drivetrain.SingleFalconDrivetrain;
import frc.robot.subsystems.drivetrain.SparkMaxDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveModuleSetupInfo;
import frc.robot.subsystems.drivetrain.commands.DisableBrakeMode;
import frc.robot.subsystems.drivetrain.commands.EnableBrakeMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.commands.DeployElevator;
import frc.robot.subsystems.elevator.commands.ManualElevatorForward;
import frc.robot.subsystems.elevator.commands.ManualElevatorReverse;
import frc.robot.subsystems.elevator.commands.MoveElevator;
import frc.robot.subsystems.elevator.commands.PlaceCone;
import frc.robot.subsystems.elevator.commands.PlaceCube;
import frc.robot.subsystems.elevator.commands.RetractElevator;
import frc.robot.subsystems.elevator.commands.ScoreConeHigh;
import frc.robot.subsystems.elevator.commands.ScoreConeMiddle;
import frc.robot.subsystems.elevator.commands.ScoreCubeHigh;
import frc.robot.subsystems.elevator.commands.ScoreCubeMiddle;
import frc.robot.telemetry.OzoneImu;
import frc.robot.telemetry.Pigeon;
import frc.robot.telemetry.Pigeon2;
import frc.robot.telemetry.commands.AutoBalance;
import frc.robot.telemetry.commands.DriveBackOntoChargeStation;
import frc.robot.telemetry.commands.DriveOntoChargeStation;
import frc.robot.telemetry.commands.DriveOverChargeStation;

/**
 * This class instantiates and initializes all of the subsystems and stores
 * references to them.
 */
public class SubsystemManager {

  private Logger logger = Logger.getLogger("Subsystem Factory");
  private BotType botType;
  private OzoneImu imu;
  private SwerveDrivetrain drivetrain;
  private PowerDistribution pdp;
  private ApriltagDetection detector;
  private Limelight limelight;

  private ActiveIntake activeIntake;
  private Claw claw;
  private ClawPitch clawPitch;
  private Elevator elevator;
  private AutonPaths paths;
  private AutoDashboardManager autoDashboardManager;

  /**
   * Map of known bot addresses and respective types
   */
  private static Map<String, BotType> allMACs = Map.of(
      "00:80:2F:30:DB:F8", BotType.COVID,
      "00:80:2F:30:DB:F9", BotType.COVID,
      "00:80:2F:25:B4:CA", BotType.RAPID_REACT,
      "00:80:2F:28:64:39", BotType.RIO99,
      "00:80:2F:28:64:38", BotType.RIO99,
      "00:80:2F:35:54:1E", BotType.CHARGED_UP_PROTO,
      "00:80:2F:17:D7:4B", BotType.RIO2,
      "", BotType.CHARGED_UP_PROTO_2,
      "00:80:2F:27:1D:E9", BotType.BLUE);

  // Should not be used outside of this class!
  private SubsystemManager() {
  }

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemManager instance;

  /**
   * Get the instance of the SubsystemFactory class.
   * If there is no instance, create one, otherwise return the already-existing
   * one.
   * 
   * @return the instance of SubsystemFactory
   */
  public static SubsystemManager getInstance() {
    if (instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }

  // ---------------------------------------------------
  // Initialization methods
  // ---------------------------------------------------

  /**
   * Create and initialize all of the subsystems.
   */
  public void init() {
    pdp = new PowerDistribution(1, ModuleType.kRev);
    botType = getBotType();

    switch (botType) {
      case COVID:
        initCOVID();
        break;
      case RAPID_REACT:
        initRAPID_REACT();
        break;
      case BLUE:
        initBLUE();
        break;
      case RIO99:
        initRIO99();
        break;
      case CHARGED_UP_PROTO_2:
        initCHARGED_UP_PROTO_2();
        break;
      default:
        if (Robot.isSimulation()) {
          initCHARGED_UP_PROTO_2();
        }
        logger.info("Unrecognized bot");
    }
  }

  private void initCHARGED_UP_PROTO_2() {
    imu = new Pigeon2(1);
    imu.reset();

    // Create and initialize all subsystems:
    drivetrain = new SingleFalconDrivetrain();
    drivetrain.init(new SwerveModuleSetupInfo[] {
        new SwerveModuleSetupInfo(31, 15, 0, 260.68),
        new SwerveModuleSetupInfo(30, 6, 3, 327.65),
        new SwerveModuleSetupInfo(32, 62, 1, 44.85),
        new SwerveModuleSetupInfo(33, 14, 2, 175.04),
    }, 1 / 8.07);

    claw = new Claw(61, 1, 0, 3, 2);
    clawPitch = new ClawPitch(7);
    activeIntake = new ActiveIntake(5, 9, 5, 4, 9);
    elevator = new Elevator(42, 6, 7);

    
    detector = new ApriltagDetection();
    detector.init();
    
    // limelight = new Limelight();

    

    // Main driver controls:
    IO io = IO.getInstance();

    Shuffleboard.getTab("Intake Control").add("deploy", Commands.runOnce(activeIntake::deploy));
    Shuffleboard.getTab("Intake Control").add("retract", Commands.runOnce(activeIntake::retract));
    Shuffleboard.getTab("Intake Control").add("off", Commands.runOnce(activeIntake::setOff));

    // Intake
    io.bind(ButtonActionType.WHEN_HELD, ControllerButton.RightTriggerButton, 
      // Commands.sequence(Commands.parallel(new DeployIntake(activeIntake), new StartIntake(activeIntake)))
      // Commands.sequence(Commands.parallel(Commands.runOnce(activeIntake::setOff), new StartIntake(activeIntake)))
      Commands.sequence(
        new DeployIntake(activeIntake),
        new StartIntake(activeIntake),
        new ParallelCommandGroup(
          new MoveElevator(elevator, ElevatorPosition.LOW),
          new SequentialCommandGroup(
            new WaitCommand(0.3),
            Commands.runOnce((activeIntake::setOff))
          )
        )
      )
      // new DeployIntake(activeIntake).andThen(new StartIntake(activeIntake)).andThen(new MoveElevator(elevator, ElevatorPosition.LOW))
    );

    io.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RightTriggerButton, 
      new StopIntake(activeIntake).andThen(new RetractIntake(activeIntake))
    );

    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton, 
      new RetractIntake(activeIntake).andThen(new ReverseIntake(activeIntake))
    );

    io.bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftTriggerButton, 
      new StopIntake(activeIntake).andThen(new RetractIntake(activeIntake))
    );


    // Reach high
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper, Commands.either(
      new ScoreConeHigh(elevator, claw, clawPitch, activeIntake),
      new ScoreCubeHigh(elevator, claw, clawPitch, activeIntake),
      activeIntake::nextPieceIsCone
    ));

    // Reach mid
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftBumper, Commands.either(
      new ScoreConeMiddle(elevator, claw, clawPitch, activeIntake),
      new ScoreCubeMiddle(elevator, claw, clawPitch, activeIntake),
      activeIntake::nextPieceIsCone
    ));

    // Place Game Piece
    /*io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A, Commands.either(
      new PlaceCone(elevator, claw, clawPitch, activeIntake),
      new PlaceCube(elevator, claw, clawPitch, activeIntake),
      activeIntake::nextPieceIsCone
    ));
    */

    // Zero Gyro
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new InstantCommand(() -> {
      imu.reset();
      imu.resetPitch();
      imu.resetRoll();
    }));

    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp, new SetClawPosition(claw, ClawPosition.OPEN));
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialDown, new SetClawPosition(claw, ClawPosition.CLOSED));

    // Brake Mode
    /*io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B, Commands.either(
      new DisableBrakeMode(drivetrain),
      new EnableBrakeMode(drivetrain),
      drivetrain::isInBrakeMode
    ));
    */

    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Back, new EmergencyCommandCancel(elevator));

    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X, new DriveOverChargeStation(drivetrain));
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A, new DriveBackOntoChargeStation(drivetrain));
    io.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B, new AutoBalance(drivetrain));

    // Aux Driver Controls

    IO aux = IO.getAuxInstance();

    aux.bind(ButtonActionType.WHEN_HELD, ControllerButton.leftYPos, new ManualElevatorForward(elevator));
    aux.bind(ButtonActionType.WHEN_HELD, ControllerButton.leftYNeg, new ManualElevatorReverse(elevator));

    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightTriggerButton, new DeployElevator(elevator));
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton, new RetractElevator(elevator));

    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Start, new EmergencyCommandCancel(elevator));

    aux.bind(ButtonActionType.WHEN_HELD, ControllerButton.A, Commands.startEnd(
      () -> activeIntake.setForceBeamBreak(true),
      () -> activeIntake.setForceBeamBreak(false)
    ));

    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new SetClawPosition(claw, ClawPosition.OPEN));
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X, new SetClawPosition(claw, ClawPosition.CLOSED));

    // Toggle Claw Angle
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B, Commands.either(
      new RotateClawToAngle(claw, Rotation2d.fromDegrees(0)),
      new RotateClawToAngle(claw, Rotation2d.fromDegrees(180)),
      () -> claw.getWristAngle().getDegrees() >= 90
    ));

    // Claw Pitch
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper, new RotateClawPitch(clawPitch, Rotation2d.fromDegrees(115)));
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftBumper, new RotateClawPitch(clawPitch, Rotation2d.fromDegrees(0)));

    aux.bind(ButtonActionType.WHEN_HELD, ControllerButton.rightXPos, new ManualClawForwards(claw));
    aux.bind(ButtonActionType.WHEN_HELD, ControllerButton.rightXNeg, new ManualClawBackwards(claw));

    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialLeft, Commands.runOnce(() -> activeIntake.setIsNextPieceCone(true)));
    aux.bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialRight, Commands.runOnce(() -> activeIntake.setIsNextPieceCone(false)));

    Shuffleboard.getTab("Auton").add("KILL PITCH", Commands.runOnce(() -> clawPitch.setKillPitch(true)));
    Shuffleboard.getTab("Auton").add("REVIVE PITCH", Commands.runOnce(() -> clawPitch.setKillPitch(false)));
  }

  private void initCHARGED_UP_PROTO() {}

  private void initBLUE() {

    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X, new autoBalancePitch(drivetrain));

    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialRight, new DeployElevator(elevator));
    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialLeft, new RetractElevator(elevator));
  }
  
  /**
   * Initializes COVID subsystems
   * 
   * @throws Exception
   */
  public void initCOVID() {
    imu = new Pigeon(21);
    imu.reset();

    drivetrain = new SparkMaxDrivetrain();
    drivetrain.init(new SwerveModuleSetupInfo[] {
        new SwerveModuleSetupInfo(34, 35, 0, 22.4),
        new SwerveModuleSetupInfo(33, 32, 1, 147.75),
        new SwerveModuleSetupInfo(37, 36, 2, 319.5),
        new SwerveModuleSetupInfo(30, 31, 3, 159.65),
    }, 1 / 8.33);

    // paths = new AutonPaths(drivetrain);

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new InstantCommand(imu::reset));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp, new EnableBrakeMode(drivetrain));
    IO.getInstance().bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RadialUp, new DisableBrakeMode(drivetrain));

    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.A, new autoBalancePitchGroup());
    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X, new driveUp(drivetrain));
    // IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B, new driveOverStation(drivetrain));
  }

  /**
   * Initializes Califorinia Bot subsystems
   * 
   * @throws Exception
   */
  public void initRAPID_REACT() {
  }

  /**
   * Initializes the RIO99 subsystems
   */
  public void initRIO99() {
  }

  /**
   * Initializes the RIO2 subsystems
   */
  public void initRIO2() {
  }

  /**
   * Initializes the RIO3 subsystems
   */
  public void initRIO3() {
  }

  // ---------------------------------------------------
  // Subsystem getter methods
  // ---------------------------------------------------

  /**
   * @return The Power Distribution Panel
   */
  public PowerDistribution getPdp() {
    return pdp;
  }

  public OzoneImu getImu() {
    return imu;
  }

  public SwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public ApriltagDetection getDetector() {
    return detector;
  }

  public Claw getClaw() {
    return claw;
  }

  public ClawPitch getClawPitch() {
    return clawPitch;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public ActiveIntake getActiveIntake() {
    return activeIntake;
  }

  public AutonPaths getAutonPaths() {
    return paths;
  }

  public AutoDashboardManager getAutoDashboardManager() {
    return autoDashboardManager;
  }

  // ---------------------------------------------------
  // Logic for determining bot type
  // ---------------------------------------------------

  /**
   * @return The active bot type
   * @throws Exception
   */
  private BotType getBotType() {
    try {
      Enumeration<NetworkInterface> networks;
      networks = NetworkInterface.getNetworkInterfaces();
      BotType bot = BotType.UNRECOGNIZED;
      for (NetworkInterface net : Collections.list(networks)) {
        String mac = formatMACAddress(net.getHardwareAddress());
        logger.info("Network #" + net.getIndex() + " " + net.getName() + " " + mac);
        if (allMACs.containsKey(mac)) {
          bot = allMACs.get(mac);
          logger.info("   this MAC is for " + bot);
        }
      }

      return bot;
    } catch (SocketException ex) {
      return BotType.UNRECOGNIZED;
    }
  }

  /**
   * Formats the byte array representing the mac address as more human-readable
   * form
   * 
   * @param hardwareAddress byte array
   * @return string of hex bytes separated by colons
   */
  private String formatMACAddress(byte[] hardwareAddress) {
    if (hardwareAddress == null || hardwareAddress.length == 0) {
      return "";
    }
    StringBuilder mac = new StringBuilder(); // StringBuilder is a premature optimization here, but done as best
                                             // practice
    for (int k = 0; k < hardwareAddress.length; k++) {
      int i = hardwareAddress[k] & 0xFF; // unsigned integer from byte
      String hex = Integer.toString(i, 16);
      if (hex.length() == 1) { // we want to make all bytes two hex digits
        hex = "0" + hex;
      }
      mac.append(hex.toUpperCase());
      mac.append(":");
    }
    mac.setLength(mac.length() - 1); // trim off the trailing colon
    return mac.toString();
  }

  /**
   * Known bots to prevent user error
   */
  public enum BotType {
    COVID,
    RAPID_REACT,
    BLUE,
    RIO99,
    CHARGED_UP_PROTO,
    CHARGED_UP_PROTO_2,
    RIO2,
    UNRECOGNIZED,
  }
}
