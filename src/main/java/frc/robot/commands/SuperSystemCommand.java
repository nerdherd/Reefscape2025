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
    private final double wristHeight;
    private final double wristAngle;
    private final int exeOrder;
    private final double timeoutSec;

    private final static int ALL_TOGETHER = 0;
    private final static int ELV_PVT_WRT = 123;
    private final static int PVT_ELV_WRT = 213;
    private final static int WRT_ELV_PVT = 312;
    private final static int WRT_PVT_ELV = 321;

    private boolean isStarted = false;
    private double startTime = 0;

    double pivotAngleRaw = 0;
    double elevatorHeightRaw = 0;
    double wristAngleRaw = 0;

    public SuperSystemCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double wristHeight, double wristAngle,
                              int exeOrder, double timeoutSec) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.wristHeight = wristHeight;
        this.wristAngle = wristAngle;
        this.exeOrder = exeOrder;
        this.timeoutSec = timeoutSec; 
        addRequirements(pivot, elevator, wrist);
    }

    @Override
    public void initialize() {
        isStarted = false;
        startTime = 0;
        //calculate pivot, elevator, wrist motors position
        updateDependencies();
    }

    @Override
    public void execute()
    {
        if(!isStarted)
        {
            isStarted = true;
            startTime = Timer.getFPGATimestamp();
        }

        if(ALL_TOGETHER == exeOrder)
        {
            //enable all
            Elevator.enabled = true;
            IntakeWrist.enabled = true;
            ElevatorPivot.enabled = true;
        }
        else{
            //TODO
        }
    }

    public void updateDependencies() 
    {
        // TODO 
        double p = pivot.getPositionRev();
        double w = wrist.getPosition();
        double e = elevator.getPosition();
        
        double pivotAngleRaw = wristAngle-wristAngle;
        double elevatorHeightRaw = wristHeight-wristHeight;
        double wristAngleRaw = wristAngle-wristAngle;

        pivot.setTargetAngleRaw(pivotAngleRaw);
        elevator.setTargetHeightRaw(elevatorHeightRaw);
        wrist.setTargetAngleRaw(wristAngleRaw);
    }

    @Override
    public void end(boolean interrupted) {
        //disable all
        Elevator.enabled = false;
        IntakeWrist.enabled = false;
        ElevatorPivot.enabled = false;
    }

    @Override
    public boolean isFinished() 
    {
        boolean timeout = ((Timer.getFPGATimestamp() - startTime) > timeoutSec); 
        boolean pidDone = (pivot.hasReachedPosition(pivotAngleRaw) && 
        elevator.hasReachedPosition(elevatorHeightRaw) && 
        wrist.hasReachedPosition(wristAngleRaw));

        return timeout || pidDone;
    }
}