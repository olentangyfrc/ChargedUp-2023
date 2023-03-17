package frc.robot.subsystems.claw.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.activeintake.ActiveIntake;
import frc.robot.subsystems.activeintake.commands.RetractIntake;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Claw.ClawPosition;
import frc.robot.subsystems.claw.ClawPitch;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.commands.MoveElevator;

public class GrabGamePiece extends SequentialCommandGroup {
    public GrabGamePiece(Claw claw, ClawPitch clawPitch, Elevator elevator, ActiveIntake intake, boolean isCone) {
        addCommands(
                // new RotateClawToAngle(claw, new Rotation2d(0)),
                new InstantCommand(() -> System.out.println("GRABBING PIECE")),
                new RetractIntake(intake),
                new ParallelCommandGroup(
                        new SetClawPosition(claw, ClawPosition.OPEN),
                        new RotateClawPitch(clawPitch, Rotation2d.fromDegrees(0))),
                        new SequentialCommandGroup(
                            new WaitCommand(0.3),
                            new RotateClawToAngle(claw, Rotation2d.fromDegrees(0))
                        ),
                new MoveElevator(elevator, (isCone ? ElevatorPosition.GRAB_CONE : ElevatorPosition.GRAB_CUBE)),
                new SetClawPosition(claw, ClawPosition.CLOSED),
                new WaitCommand(0.3),
                new MoveElevator(elevator, ElevatorPosition.LOW)
        );

        if(isCone) {
            addCommands(
                new ParallelCommandGroup(
                    new RotateClawToAngle(claw, Rotation2d.fromDegrees(180)),
                    new RotateClawPitch(clawPitch, Rotation2d.fromDegrees(115))
                )
            );
        }
    }

}
