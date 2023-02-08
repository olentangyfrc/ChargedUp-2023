package frc.robot.subsystems.prototypeone.intakeArm.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.prototypeone.intakeArm.intakeArm;

public class toggleClaw extends InstantCommand {
    private intakeArm intakeArm;
    public toggleClaw(intakeArm intakeArm){
        this.intakeArm = intakeArm;
    }

    @Override
    public void initialize(){
        if(intakeArm.isClawClosed()){
            intakeArm.openClaw();
        }
        else {
            intakeArm.closeClaw();
        }
    }
}
