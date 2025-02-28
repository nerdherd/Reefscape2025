package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StowCommand extends SequentialCommandGroup{
    private final ElevatorPivot pivot;
    private final Elevator elevator;
    private final IntakeWrist wrist;
    private final double pivotAngle;
    private final double elevatorHeight;
    private final double wristAngle;

    private final static int ALL_TOGETHER = 0;
    private final static int ELV_PVT_WRT = 123;
    private final static int PVT_ELV_WRT = 213;
    private final static int WRT_ELV_PVT = 312;
    private final static int WRT_PVT_ELV = 321;

    private boolean isStarted = false;
    private int currentTime = 0;

    private int exeOrder;

    public StowCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double pivotAngle, double elevatorHeight, double wristAngle,
                              int exeOrder, int timeout) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;
        this.exeOrder = exeOrder;
        this.wristAngle = wristAngle;
        addRequirements(pivot, elevator, wrist);
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

    // @Override
    // public void initialize() {
    //     pivot.setEnabled(true);
    //     wrist.setEnabled(false);
    //     elevator.setEnabled(false);
    //     pivot.setTargetPosition(pivotAngle);
    //     elevator.setPosition(elevatorHeight);
    //     wrist.setPosition(wristAngle);
    // }

    // @Override
    // public void execute()
    // {
    //     double lastTime = Timer.getFPGATimestamp();
    //     updateDependencies(); 
    //     pivot.setTargetPosition(pivotAngle);

    //     // if(Timer.getFPGATimestamp() - lastTime > 4.0) {
    //     //     wrist.setPosition(-0.8);
    //     // }

    //     if(pivot.atPosition()) {
    //         wrist.setEnabled(true);
    //         if(wrist.atPosition()) {
    //             elevator.setEnabled(true);
    //         }
    //     }
    //      elevator.setPositionCommand(elevatorHeight);
    //     wrist.setPositionCommand(wristAngle);
    // }

    // public void updateDependencies() { 
    //     double curPivotAngle = pivot.getPositionDegrees();
    //     // pivot.setTargetPosition(elevator.getPosition()); 
    //     elevator.setPivotAngle(curPivotAngle);
    //     wrist.setPivotAngle(curPivotAngle);
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     pivot.stop();
    //     elevator.stop();
    //     wrist.stop();
    // }

    // @Override
    // public boolean isFinished() {
    //     return (pivot.atSetpoint() && elevator.atSetpoint() && wrist.atSetpoint());
    // }
}