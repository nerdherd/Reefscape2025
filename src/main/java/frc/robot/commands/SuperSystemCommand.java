package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.util.NerdyMath;

public class SuperSystemCommand extends Command{
    private final ElevatorPivot pivot;
    private final Elevator elevator;
    private final IntakeWrist wrist;
    private final double pivotAngle;
    private final double elevatorHeight;
    private final double wristAngle;
    private final BooleanSupplier pivotAtPosition;
    private final BooleanSupplier elevatorAtPosition;
    private final BooleanSupplier wristAtPosition;
    private int ammountCalled = 0;

    public enum ExecutionOrder {
        ALL_TOGETHER,
        ELV_PVT_WRT,
        ELV_WRT_PVT,
        PVT_ELV_WRT,
        PVT_WRT_ELV,
        WRT_ELV_PVT,
        WRT_PVT_ELV
    }

    private boolean isStarted = false;
    private double startTime = 0;
    private double timeout;

    private ExecutionOrder exeOrder;
    // rotations for angles
    public SuperSystemCommand(ElevatorPivot pivot, Elevator elevator, IntakeWrist wrist,
                              double pivotAngle, double elevatorHeight, double wristAngle,
                              ExecutionOrder wrtElvPvt, double timeout, BooleanSupplier pivotAtPosition, BooleanSupplier elevatorAtPosition, BooleanSupplier wristAtPosition) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;
        this.exeOrder = wrtElvPvt;
        this.wristAngle = wristAngle;
        this.pivotAtPosition = pivotAtPosition;
        this.elevatorAtPosition = elevatorAtPosition;
        this.wristAtPosition = wristAtPosition;
        addRequirements(pivot, elevator, wrist);
    }

    @Override
    public void initialize() {
        pivot.setEnabled(false);
        wrist.setEnabled(false);
        elevator.setEnabled(false);
        pivot.setTargetPosition(pivotAngle);
        elevator.setPosition(elevatorHeight);
        wrist.setPosition(wristAngle);
        ammountCalled = 0;
    }

    @Override
    public void execute()
    {
        if(!isStarted) {
            isStarted = true;
            startTime = Timer.getFPGATimestamp();
        }

        updateDependencies(); 

        switch (exeOrder) {
            case ALL_TOGETHER:
                pivot.setEnabled(true);
                wrist.setEnabled(true);
                elevator.setEnabled(true);
                break;

            case ELV_PVT_WRT:
                elevator.setEnabled(true);
                if(elevator.atPosition()) {
                    pivot.setEnabled(true);
                }
                if(pivot.atPosition()) {
                    wrist.setEnabled(true);
                }
                break;

            case ELV_WRT_PVT:
                elevator.setEnabled(true);
                if(elevator.atPosition()) {
                    wrist.setEnabled(true);
                }
                if(wrist.atPosition()) {
                    pivot.setEnabled(true);
                }
                break;

            case PVT_WRT_ELV:
                pivot.setEnabled(true);
                if(pivot.atPosition()) {
                    wrist.setEnabled(true);
                }
                if(wrist.atPosition()) {
                    elevator.setEnabled(true);
                }
                break;

            case PVT_ELV_WRT:
                pivot.setEnabled(true);
                if(pivot.atPosition()) {
                    elevator.setEnabled(true);
                }
                if(elevator.atPosition()) {
                    wrist.setEnabled(true);
                }
                break;

            case WRT_ELV_PVT:
                wrist.setEnabled(true);
                if(wrist.atPosition()) {
                    elevator.setEnabled(true);
                }
                if(elevator.atPosition()) {
                    pivot.setEnabled(true);
                }
                break;

            case WRT_PVT_ELV:
                ammountCalled += 1;
                SmartDashboard.putNumber("Ammount Called", ammountCalled);
                wrist.setEnabled(true);
                SmartDashboard.putBoolean("Wrist at Position", false);
                if(wristAtPosition.getAsBoolean()) {
                    SmartDashboard.putBoolean("Wrist at Position", true);
                    pivot.setEnabled(true);
                }

                SmartDashboard.putBoolean("Pivot at Position", false);
                if(pivotAtPosition.getAsBoolean()) {
                    SmartDashboard.putBoolean("Pivot at position", false);
                    elevator.setEnabled(true);
                }
                break;
        
            default:
                break;
        }
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPositionDegrees() / 360.0;
        // pivot.setTargetPosition(elevator.getPosition()); 
        elevator.setPivotAngle(curPivotAngle);
        wrist.setPivotAngle(curPivotAngle);
    }

    //TODO: see if we want to stop at the end of the command, maybe not bcuz we might want to hold it there
    @Override
    public void end(boolean interrupted) {
        // pivot.stop();
        // elevator.stop();
        // wrist.stop();
    }

    @Override
    public boolean isFinished() {
        boolean isFinished = (pivotAtPosition.getAsBoolean() && elevatorAtPosition.getAsBoolean() && wristAtPosition.getAsBoolean()) || (Timer.getFPGATimestamp() - startTime > timeout);
        SmartDashboard.putBoolean("Is Finished", isFinished);
        return isFinished;
    }
}