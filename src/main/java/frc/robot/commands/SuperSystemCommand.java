package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;

public class SuperSystemCommand extends Command {
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

    public SuperSystemCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double pivotAngle, double elevatorHeight, double wristAngle,
                              int exeOrder, int timeout) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;
        addRequirements(pivot, elevator, wrist);
    }

    @Override
    public void initialize() {
        pivot.setTargetPosition(pivotAngle);
        //elevator.setPosition(elevatorHeight);
        wrist.setPosition(wristAngle);
        pivot.setEnabledCommand(true);
        wrist.setEnabledCommand(true);
    }

    @Override
    public void execute()
    {
        double lastTime = Timer.getFPGATimestamp();
        updateDependencies(); 
        pivot.setTargetPosition(pivotAngle);
         //elevator.setPositionCommand(elevatorHeight);
        wrist.setPositionCommand(wristAngle);
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPositionDegrees();
        // pivot.setTargetPosition(elevator.getPosition()); 
        elevator.setPivotAngle(curPivotAngle);
        wrist.setPivotAngle(curPivotAngle);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stop();
        elevator.stop();
        wrist.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return (pivot.atSetpoint() && elevator.atSetpoint() && wrist.atSetpoint());
    // }
}