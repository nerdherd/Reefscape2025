package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StowCommand extends SequentialCommandGroup{
    public StowCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double pivotAngle, double elevatorHeight, double wristAngle,
                               int timeout) {
        addCommands(
            Commands.parallel(
                Commands.sequence(
                    elevator.setPivotAngleCommand(pivot.getPositionDegrees()),
                    wrist.setPivotAngleCommand(pivot.getPositionDegrees())
                ),
                Commands.sequence(
                        wrist.setPositionCommand(wristAngle - 0.4),
                        wrist.setEnabledCommand(true),

                        Commands.waitSeconds(0.5),
                        elevator.setPositionCommand(elevatorHeight),
                        elevator.setEnabledCommand(true),

                        wrist.setPositionCommand(wristAngle),

                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() ->pivot.setTargetPosition(pivotAngle)),
                        pivot.setEnabledCommand(true)
                        
                )
            )
        );
    }
}