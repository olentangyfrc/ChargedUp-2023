package frc.robot.subsystems.prototypeone.intakeArm.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.prototypeone.intakeArm.intakeArm;

import java.util.logging.Logger;

public class armDown extends InstantCommand{
    private intakeArm intakeArm;

    private static Logger logger = Logger.getLogger(armDown.class.getName());

    public armDown(intakeArm intakeArm) {
        this.intakeArm = intakeArm;
    }

    @Override
    public void initialize(){
        logger.info("Putting Arm Down");
        intakeArm.retractArm();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
