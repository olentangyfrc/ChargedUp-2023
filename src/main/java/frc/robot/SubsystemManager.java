package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.IO.ButtonActionType;
import frc.robot.IO.ControllerButton;
import frc.robot.auton.AutoDashboardManager;
import frc.robot.auton.AutoNodeUtility;
import frc.robot.auton.AutonPaths;
import frc.robot.subsystems.ApriltagDetection;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.DeployIntake;
import frc.robot.subsystems.activeintake.commands.RetractIntake;
import frc.robot.subsystems.activeintake.commands.ReverseIntake;
import frc.robot.subsystems.activeintake.commands.StartIntake;
import frc.robot.subsystems.activeintake.commands.StopIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.claw.commands.SetClawPosition;
import frc.robot.subsystems.drivetrain.SingleFalconDrivetrain;
import frc.robot.subsystems.drivetrain.SparkMaxDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;
import frc.robot.subsystems.drivetrain.SwerveModuleSetupInfo;
import frc.robot.subsystems.drivetrain.commands.DisableBrakeMode;
import frc.robot.subsystems.drivetrain.commands.EnableBrakeMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.DeployElevator;
import frc.robot.subsystems.elevator.commands.ManualElevatorForward;
import frc.robot.subsystems.elevator.commands.ManualElevatorReverse;
import frc.robot.subsystems.elevator.commands.MoveElevator;
import frc.robot.subsystems.elevator.commands.RetractElevator;
import frc.robot.subsystems.elevator.commands.ScoreHigh;
import frc.robot.subsystems.elevator.commands.ScoreMiddle;
import frc.robot.subsystems.prototypeone.elevator.ProtoElevator;
import frc.robot.subsystems.prototypeone.intakeArm.intakeArm;
import frc.robot.subsystems.prototypeone.intakeArm.commands.armDown;
import frc.robot.subsystems.prototypeone.intakeArm.commands.armUp;
import frc.robot.subsystems.prototypeone.intakeArm.commands.toggleClaw;
import frc.robot.telemetry.OzoneImu;
import frc.robot.telemetry.Pigeon;
import frc.robot.telemetry.Pigeon2;

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
  private intakeArm intakeArm;
  private ProtoElevator protoElevator;

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
    pdp = new PowerDistribution(52, ModuleType.kCTRE);
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
      case CHARGED_UP_PROTO:
        initCHARGED_UP_PROTO();
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
        new SwerveModuleSetupInfo(31, 15, 0, 261.52),
        new SwerveModuleSetupInfo(30, 6, 2, 328.8),
        new SwerveModuleSetupInfo(32, 62, 1, 41.36),
        new SwerveModuleSetupInfo(33, 14, 3, 180.48),
    }, 1 / 8.07);

    claw = new Claw(61, 0, 1, 2, 3);
    clawPitch = new ClawPitch(7);
    activeIntake = new ActiveIntake(5, 9, 5, 4);
    elevator = new Elevator(42, 6, 7);

    Shuffleboard.getTab("Command Groups").add("Score middle", new ScoreMiddle(elevator, claw, clawPitch, activeIntake));
    Shuffleboard.getTab("Command Groups").add("Score High", new ScoreHigh(elevator, claw, clawPitch, activeIntake));

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new InstantCommand(imu::reset));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightTriggerButton,
        new StartIntake(activeIntake));
    IO.getInstance().bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RightTriggerButton,
        new StopIntake(activeIntake));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton,
        new ReverseIntake(activeIntake));
    IO.getInstance().bind(ButtonActionType.WHEN_RELEASED, ControllerButton.LeftTriggerButton,
        new StopIntake(activeIntake));

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper, new DeployIntake(activeIntake));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftBumper, new RetractIntake(activeIntake));

    IO.getInstance().bind(ButtonActionType.WHEN_HELD, ControllerButton.A, new ManualElevatorForward(elevator));
    IO.getInstance().bind(ButtonActionType.WHEN_HELD, ControllerButton.B, new ManualElevatorReverse(elevator));

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp,
        new SetClawPosition(claw, ClawPosition.CLOSED));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialDown,
        new SetClawPosition(claw, ClawPosition.LOWER_LATCH));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.X,
        new SetClawPosition(claw, ClawPosition.OPEN));

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialRight, new DeployElevator(elevator));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialLeft, new RetractElevator(elevator));
  }

  private void initCHARGED_UP_PROTO() {

    imu = new Pigeon2(5);
    imu.reset();

    // Create and initialize all subsystems:
    drivetrain = new SingleFalconDrivetrain();
    drivetrain.init(new SwerveModuleSetupInfo[] {
        new SwerveModuleSetupInfo(41, 59, 1, 331.6),
        new SwerveModuleSetupInfo(40, 8, 3, 28.39),
        new SwerveModuleSetupInfo(42, 17, 2, 28.87),
        new SwerveModuleSetupInfo(43, 15, 0, 267.34),
    }, 1 / 8.07);
    detector = new ApriltagDetection();

    protoElevator = new ProtoElevator();
    paths = new AutonPaths(drivetrain);
    autoDashboardManager = new AutoDashboardManager();

    detector.init();
    // elevator = new Elevator();

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new InstantCommand(imu::reset));

    intakeArm = new intakeArm();
    intakeArm.init();
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.B, new toggleClaw(intakeArm));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftBumper, new armDown(intakeArm));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightBumper, new armUp(intakeArm));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RightTriggerButton,
        new MoveElevator(elevator, ProtoElevator.ELEVATOR_HIGH_POS));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.LeftTriggerButton,
        new MoveElevator(elevator, 0));

    IO.getInstance().bind(ButtonActionType.WHEN_HELD, ControllerButton.Start, Commands.run(
        () -> paths.pathToPositionCommand(AutoNodeUtility.getNodeDrivePosition(autoDashboardManager.getSelectedNode()))
            .schedule()));
  }

  private void initBLUE() {
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

    paths = new AutonPaths(drivetrain);

    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.Y, new InstantCommand(imu::reset));
    IO.getInstance().bind(ButtonActionType.WHEN_PRESSED, ControllerButton.RadialUp, new EnableBrakeMode(drivetrain));
    IO.getInstance().bind(ButtonActionType.WHEN_RELEASED, ControllerButton.RadialUp, new DisableBrakeMode(drivetrain));
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

  public ProtoElevator getProtoElevator() {
    return protoElevator;
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
