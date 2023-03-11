package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.commands.MoveElevator;

public class GrabGamePiece extends SequentialCommandGroup {
    public GrabGamePiece(Claw claw, ClawPitch clawPitch, Elevator elevator, boolean isCone) {
        addCommands(
                // new RotateClawToAngle(claw, new Rotation2d(0)),
                new ParallelCommandGroup(
                        new SetClawPosition(claw, ClawPosition.OPEN),
                        new RotateClawToAngle(claw, Rotation2d.fromDegrees(0)),
                        new RotateClawPitch(clawPitch, Rotation2d.fromDegrees(0))),
                new MoveElevator(elevator, (isCone ? ElevatorPosition.GRAB_CONE : ElevatorPosition.GRAB_CUBE)),
                new SetClawPosition(claw, ClawPosition.CLOSED));
    }

}
