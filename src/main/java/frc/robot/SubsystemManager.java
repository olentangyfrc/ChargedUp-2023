package frc.robot;

import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Map;
import java.util.logging.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.telemetry.Pigeon;

/**
 * This class instantiates and initializes all of the subsystems and stores references to them.
 */
public class SubsystemManager {

  private Logger logger = Logger.getLogger("Subsystem Factory");
  private BotType botType;
  private Pigeon pigeon;
  private PowerDistribution pdp;

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
    "00:80:2F:27:04:C6", BotType.RIO3,
    "00:80:2F:27:1D:E9", BotType.BLUE
  );

  // Should not be used outside of this class!
  private SubsystemManager() {}

  // SubsystemFactory is a singleton, so keep a static instance.
  private static SubsystemManager instance;
  
  /**
   * Get the instance of the SubsystemFactory class.
   * If there is no instance, create one, otherwise return the already-existing one.
   * 
   * @return the instance of SubsystemFactory
   */
  public static SubsystemManager getInstance() {
    if(instance == null) {
      instance = new SubsystemManager();
    }
    return instance;
  }
  

  //---------------------------------------------------
  // Initialization methods
  //---------------------------------------------------

  /**
   * Create and initialize all of the subsystems.
   */
  public void init() throws Exception {
    pdp = new PowerDistribution(52, ModuleType.kCTRE);
    botType = getBotType();

    switch(botType) {
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
      default:
        logger.info("Unrecognized bot");
    }
  }

  private void initCHARGED_UP_PROTO() throws Exception {
    pigeon = new Pigeon(5);
    System.out.println("Init CHARGED_UP_PROTO");
    // Shuffleboard.getTab("Gyro").addNumber("Yaw", pigeon::getAngle);
    // Shuffleboard.getTab("Gyro").addNumber("Roll", pigeon::getRoll);
    // Shuffleboard.getTab("Gyro").addNumber("Pitch", pigeon::getPitch);
    // Shuffleboard.getTab("Gyro").add("Reset Zero", new InstantCommand(pigeon::reset));
    // Shuffleboard.getTab("Gyro").add("reset 30", new InstantCommand(() -> pigeon.setReset(Rotation2d.fromDegrees(30))));
  }

  private void initBLUE() {}

  /**
   * Initializes COVID subsystems
   * @throws Exception
   */
  public void initCOVID() throws Exception {}

  /**
   * Initializes Califorinia Bot subsystems
   * @throws Exception
   */
  public void initRAPID_REACT() throws Exception {}

  /**
   * Initializes the RIO99 subsystems
   */
  public void initRIO99() {}

  /**
   * Initializes the RIO2 subsystems
   */
  public void initRIO2() {}

  /**
   * Initializes the RIO3 subsystems
   */
  public void initRIO3() {}
  

  //---------------------------------------------------
  // Subsystem getter methods
  //---------------------------------------------------

  /**
   * @return The Power Distribution Panel
   */
  public PowerDistribution getPdp() {
    return pdp;
  }


  //---------------------------------------------------
  // Logic for determining bot type
  //---------------------------------------------------

  /**
   * @return The active bot type
   * @throws Exception
   */
  private BotType getBotType() throws Exception {
    Enumeration<NetworkInterface> networks;
    networks = NetworkInterface.getNetworkInterfaces();
    BotType bot = BotType.UNRECOGNIZED;
    for (NetworkInterface net : Collections.list(networks)) {
        String mac = formatMACAddress(net.getHardwareAddress());
        logger.info("Network #"+net.getIndex()+" "+net.getName()+" "+mac);
        if (allMACs.containsKey(mac)) {
            bot = allMACs.get(mac);
            logger.info("   this MAC is for "+bot);
        }
    }

    return bot;
  }

  /**
   * Formats the byte array representing the mac address as more human-readable form
   * @param hardwareAddress byte array
   * @return string of hex bytes separated by colons
   */
  private String formatMACAddress(byte[] hardwareAddress) {
    if (hardwareAddress == null || hardwareAddress.length == 0) {
      return "";
    }
    StringBuilder mac = new StringBuilder(); // StringBuilder is a premature optimization here, but done as best practice
    for (int k=0;k<hardwareAddress.length;k++) {
      int i = hardwareAddress[k] & 0xFF;  // unsigned integer from byte
      String hex = Integer.toString(i,16);
      if (hex.length() == 1) {  // we want to make all bytes two hex digits 
        hex = "0"+hex;
      }
      mac.append(hex.toUpperCase());
      mac.append(":");
    }
    mac.setLength(mac.length()-1);  // trim off the trailing colon
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
    RIO2, 
    RIO3,
    UNRECOGNIZED,
  }
}
