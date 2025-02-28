package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;

public class ArmSystemCommand extends Command {
    private final ElevatorPivot pivot;
    private final Elevator elevator;
    private final IntakeWrist wrist;
    private final double wristHeight;
    private final double wristAngle;
    private ARM_POSITION positionID;
    private final int exeOrder;
    private final double timeoutSec;

    private final static int ALL_TOGETHER = 0;

    private boolean isStarted = false;
    private double startTime = 0;

    double pivotAngleRaw = 0;
    double elevatorHeightRaw = 0;
    double wristAngleRaw = 0;

    public enum ARM_POSITION
    {
        STOW,
        REEF_L1,
        REEF_L2,
        REEF_L3,
        REEF_L4,
        REEF_BALL,
        STATION,
        PROCESSOR,
        CAGE,
        FLOOR_PICK,
        UNKNOWN
    }

    public ArmSystemCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double wristHeight, double wristAngle, ARM_POSITION positionID,
                              int exeOrder, double timeoutSec) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist; // not for controlling wrist. 
        this.wristHeight = wristHeight;
        this.wristAngle = wristAngle;
        this.exeOrder = exeOrder;
        this.timeoutSec = timeoutSec; 
        this.positionID = positionID;
        addRequirements(pivot, elevator);
    }

    @Override
    public void initialize() {
        isStarted = false;
        startTime = 0;

        if(ALL_TOGETHER == exeOrder)
        {
            //enable all
            Elevator.enabled = true;
            ElevatorPivot.enabled = true;
        }
        else{
            //TODO
        }

        //calculate pivot, elevator, wrist motors position
        if(positionID == ARM_POSITION.STOW)
        {
            pivot.setTargetPosition(0.01);
            elevator.setPosition(0.01);
        }
        else if(positionID == ARM_POSITION.REEF_L1)
        {
            pivot.setTargetPosition(0.15);
            elevator.setPosition(0.7);
        }
        else if(positionID == ARM_POSITION.REEF_L4)
        {
            pivot.setTargetPosition(0.2);
            elevator.setPosition(0.9);
        }
    }

    @Override
    public void execute()
    {
        if(!isStarted)
        {
            isStarted = true;
            startTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //disable all
        Elevator.enabled = false;
        ElevatorPivot.enabled = false;
    }

    @Override
    public boolean isFinished() 
    {
        boolean timeout = ((Timer.getFPGATimestamp() - startTime) > timeoutSec); 
        boolean pidDone = (pivot.hasReachedPosition(pivotAngleRaw) && 
        elevator.hasReachedPosition(elevatorHeightRaw) 
        );

        return timeout || pidDone;
    }
}