package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;
    public IntakeV2 claw;

    private BooleanSupplier pivotAtPosition, elevatorAtPosition, wristAtPosition;

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

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, IntakeV2 claw) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;

        pivotAtPosition = () -> pivot.atPosition();
        elevatorAtPosition = () -> elevator.atPosition();
        wristAtPosition = () -> wrist.atPosition();

        initialize();
    }

    public Command zeroEncoders() {
        return Commands.runOnce(()-> {
            pivot.zeroEncoder();
            elevator.zeroEncoder();
            wrist.zeroEncoder();
            claw.zeroEncoder();
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            pivot.stop();
            wrist.stop();
            elevator.stop();
        });
    }

    public Command stopRoller() {
        return claw.setVelocityCommand(0.0);
    }

    public Command closeClaw() {
        return claw.setJawPositionCommand(ClawConstants.kClosedPosition);
    }

    public Command intakeCoral() {
        return claw.intakeCoral();
    }

    public Command outtakeCoral() {
        return claw.outtakeCoral();
    }

    public Command intakeAlgae() {
        return claw.intakeAlgae();
    }

    public Command outtakeAlgae() {
        return claw.outtakeAlgae();
    }

    public Command moveTo(NamedPositions position) {
        return Commands.sequence(
            execute(position.executionOrder, 10.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            
            wrist.setPositionCommand(position.finalWristPosition)
        );
    }

    public Command moveToStow() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kStowPosition)
        );
    }

    public Command moveToSemiStow() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotSemiStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition)
        );
    }

    public Command moveToCage() { //TODO
        return Commands.none();
    }

    public Command moveToNet() { //TODO
        return Commands.none();
    }

    public Command moveToProcs() { //TODO
        return Commands.none();
    }    

    public Command moveTogroundIntake() {
        return Commands.sequence(
            execute(ExecutionOrder.PVT_WRT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorGroundIntake, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kWristGroundIntake),
            intakeCoral(),
            Commands.race(
                Commands.waitSeconds(5),
                stopRoller()
            )
        );
    }

    public Command moveToStation() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStationPosition, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kStationPosition),
            intakeCoral(),
            Commands.race(
                Commands.waitSeconds(5),
                stopRoller()
            )
        );
    }

    public Command moveToL1() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_PVT_ELV,10.0,
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position,WristConstants.kIntermediatePosition),

            wrist.setPositionCommand(WristConstants.kWristL1Position)
        );

    }

    public Command moveToL2() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kWristL2Position)
        );
    }

    public Command moveToL3() {
    return Commands.sequence(
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position,WristConstants.kIntermediatePosition),

        wrist.setPositionCommand(WristConstants.kWristL3Position)
    );
    }

    public Command moveToL4() {
    return Commands.sequence(
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL4Position,WristConstants.kIntermediatePosition),

        wrist.setPositionCommand(WristConstants.kWristL4Position)
    );
    }

    public void initialize() {
        pivot.setEnabled(true);
        wrist.setEnabled(true);
        elevator.setEnabled(true);
        claw.setEnabled(true);
        pivot.setTargetPosition(0);
        elevator.setPosition(0);
        wrist.setPosition(0);
        claw.setJawPosition(ClawConstants.kClosedPosition);
        claw.setVelocityCommand(0.0);
        isStarted = false;
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPositionRev();
        // pivot.setTargetPosition(elevator.getPosition()); 
        elevator.setPivotAngle(curPivotAngle);
        wrist.setPivotAngle(curPivotAngle);
    }

    public Command execute(
        ExecutionOrder exeOrder, double timeout, 
        double pivotAngle, double elevatorPosition, double wristAngle
    ) {
        return Commands.runEnd(() -> {
            if(!isStarted) {
                isStarted = true;
                startTime = Timer.getFPGATimestamp();
            }
    
            updateDependencies(); 
    
            switch (exeOrder) {
                case ALL_TOGETHER:
                    pivot.setTargetPosition(pivotAngle);
                    elevator.setPosition(elevatorPosition);
                    wrist.setPosition(wristAngle);
                    break;
    
                case ELV_PVT_WRT:
                    elevator.setPosition(elevatorPosition);
                    if(elevatorAtPosition.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        if(pivotAtPosition.getAsBoolean()) {
                            wrist.setPosition(wristAngle);
                        }
                    }
                    break;
    
                case ELV_WRT_PVT:
                    elevator.setPosition(elevatorPosition);
                    if(elevatorAtPosition.getAsBoolean()) {
                        wrist.setPosition(wristAngle);
                        if(wristAtPosition.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                        }
                    }
                    break;
    
                case PVT_WRT_ELV:
                    pivot.setTargetPosition(pivotAngle);
                    if(pivotAtPosition.getAsBoolean()) {
                        wrist.setPosition(wristAngle);
                        if(wristAtPosition.getAsBoolean()) {
                            elevator.setPosition(elevatorPosition);
                        }
                    }
                    break;
    
                case PVT_ELV_WRT:
                    pivot.setTargetPosition(pivotAngle);
                    if(pivotAtPosition.getAsBoolean()) {
                        elevator.setPosition(elevatorPosition);
                        if(elevatorAtPosition.getAsBoolean()) {
                            wrist.setPosition(wristAngle);
                        }
                    }
                    break;
    
                case WRT_ELV_PVT:
                    wrist.setPosition(wristAngle);
                    if(wristAtPosition.getAsBoolean()) {
                        elevator.setPosition(elevatorPosition);
                        if(elevatorAtPosition.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                        }
                    }
                    break;
    
                case WRT_PVT_ELV:
                    wrist.setPosition(wristAngle);
                    if(wristAtPosition.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        if(pivotAtPosition.getAsBoolean()) {
                            elevator.setPosition(elevatorPosition);
                        }
                    }
                    break;
            
                default:
                    break;
            }
        },
        () -> {
            isStarted = false;
        }
        ).until(
            () -> ((pivot.atPosition() && elevator.atPosition() && wrist.atPosition()) || (Timer.getFPGATimestamp() - startTime >= timeout))
        )
        ;
    }

}
