package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.IntakeWrist;

public class StationCommand extends SequentialCommandGroup{
    public StationCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double pivotAngle, double elevatorHeight, double wristAngle,
                              double timeout) {

        addCommands(
            Commands.parallel(
                Commands.sequence(
                    elevator.setPivotAngleCommand(pivot.getPositionDegrees()),
                    wrist.setPivotAngleCommand(pivot.getPositionDegrees())
                ),
                Commands.sequence(
                        Commands.runOnce(() -> pivot.setTargetPosition(pivotAngle)),
                        pivot.setEnabledCommand(true),

                        Commands.waitSeconds(0.5),
                        wrist.setPositionCommand(wristAngle),
                        wrist.setEnabledCommand(true),

                        Commands.waitSeconds(0.5),
                        elevator.setPositionCommand(elevatorHeight),
                        elevator.setEnabledCommand(true),

                        Commands.waitSeconds(0.5),
                        wrist.setPositionCommand(wristAngle * 2)
                        
                )
            )
        );
    }
}
