package frc.robot.subsystems.intakeArm;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

public class intakeArm extends SubsystemBase{

    private Compressor compressor;

    private final int PCM_CAN_ID = 2;

    private DoubleSolenoid arm;
    private final int pnematicsArmForward = 0;
    private final int pnematicsArmBackward = 2;
   
    private DoubleSolenoid claw;
    private final int pnematicsClawForward = 1;
    private final int pnematicsClawBackward = 3;
    private boolean isClawClosed = false;

    private Logger logger = Logger.getLogger("Arm and Claw Subsystem");

    public void init() {
        compressor = new Compressor(PCM_CAN_ID, PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();
        arm = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, pnematicsArmForward, pnematicsArmBackward);
        arm.set(Value.kOff);
        claw = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, pnematicsClawForward, pnematicsClawBackward);
        claw.set(Value.kOff);
    }

    public void extendArm() {
        arm.set(Value.kForward);
    }

    public void retractArm() {
        arm.set(Value.kReverse);
    }

    public void openClaw() {
        claw.set(Value.kForward);
        isClawClosed = false;
    }

    public void closeClaw() {
        claw.set(Value.kReverse);
        isClawClosed = true;
    }

    public boolean isClawClosed(){
        return isClawClosed;
    }
}
