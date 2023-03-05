package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.commands.RotateClawToAngle;
import frc.robot.subsystems.claw.commands.SetClawPosition;

public class GrabGamePiece extends SequentialCommandGroup{
    private Claw claw;
    public GrabGamePiece(Claw claw) {
        this.claw = claw;
        addCommands(
            //new RotateClawToAngle(claw, new Rotation2d(0)),
            new SetClawPosition(claw, ClawPosition.CLOSED)
        );
    }
}
