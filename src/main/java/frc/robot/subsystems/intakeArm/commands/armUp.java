package frc.robot.subsystems.intakeArm.commands;

import frc.robot.subsystems.intakeArm.intakeArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.logging.Logger;

public class armUp extends InstantCommand{
    private intakeArm intakeArm;

    private static Logger logger = Logger.getLogger(armDown.class.getName());

    public armUp(intakeArm intakeArm) {
        this.intakeArm = intakeArm;
    }

    @Override
    public void initialize(){
        logger.info("Putting Arm Up");
    }

    @Override
    public void execute(){
        intakeArm.extendArm();
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
