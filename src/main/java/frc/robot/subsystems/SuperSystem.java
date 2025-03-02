package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.V1ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.commands.SuperSystemCommand;
import frc.robot.commands.SuperSystemCommand.ExecutionOrder;

public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;
    public IntakeV2 claw;

    public enum ExecutionOrder {
        ALL_TOGETHER,
        ELV_PVT_WRT,
        ELV_WRT_PVT,
        PVT_ELV_WRT,
        PVT_WRT_ELV,
        WRT_ELV_PVT,
        WRT_PVT_ELV
    }
    private ExecutionOrder exeOrder;

    private boolean isStarted = false;
    private int ammountCalled = 0;
    private double startTime = 0;
    private double timeout;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, IntakeV2 claw) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;

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
    //     SuperSystemCommand superSystemCommand = 
    //         new SuperSystemCommand(pivot, elevator, wrist, 
    //             position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition, 
    //             position.executionOrder, 10.0);
    //     if (position.intermediateWristPosition == position.finalWristPosition) return superSystemCommand;
    //     return Commands.sequence(
    //       superSystemCommand,
    //       wrist.setPositionCommand(position.finalWristPosition),
    //       wrist.setEnabledCommand(true)
    //     );
    return Commands.none();
    }

    public Command moveToStow() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
    //     ExecutionOrder.WRT_ELV_PVT, 10.0);

    //     return Commands.sequence(
    //         superSystemCommand,
    //         wrist.setPositionCommand(WristConstants.kStowPosition),
    //         wrist.setEnabledCommand(true)
    //     );
        return Commands.sequence(
            Commands.runOnce(() -> isStarted = false),
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition,
            () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),
            
            wrist.setPositionCommand(WristConstants.kStowPosition)
        );
    }

    public Command semiStow() {
        return Commands.sequence(
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition,
            () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),
            
            wrist.setPositionCommand(WristConstants.kStowPosition)
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


    public Command moveToSemiStow() { //TODO: see if this is legit
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition, 
    //     ExecutionOrder.WRT_ELV_PVT, 10.0,
    //     () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()
    //     );

    //     return Commands.sequence(
    //         Commands.runOnce(() -> superSystemCommand.initialize()),
    //         Commands.run(() -> superSystemCommand.execute())
    //     );
        return Commands.none();
    }

    public Command moveToStation() {
        return Commands.sequence(
            Commands.runOnce(() -> isStarted = false),
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStationPosition, WristConstants.kIntermediatePosition,
            () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),
            
            wrist.setPositionCommand(WristConstants.kStationPosition)
        );
    }

    public Command moveToL2L3() {
        return Commands.none();
    }

    public Command moveToL3L4() {
        return Commands.none();
    }

    public Command moveToL1() {
        return Commands.sequence(
            Commands.runOnce(() -> isStarted = false),
            execute(ExecutionOrder.WRT_PVT_ELV,10.0,
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position,WristConstants.kIntermediatePosition,
            () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),

            wrist.setPositionCommand(WristConstants.kWristL1Position)
        );

    }

    public Command moveToL2() {
        return Commands.sequence(
            Commands.runOnce(() -> isStarted = false),
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition,
            () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),
            
            wrist.setPositionCommand(WristConstants.kWristL2Position)
        );
    }

    public Command moveToL3() {
    return Commands.sequence(
        Commands.runOnce(() -> isStarted = false),
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position,WristConstants.kIntermediatePosition,
        () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),

        wrist.setPositionCommand(WristConstants.kWristL3Position)
    );
    }

    public Command moveToL4() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, 
    //     V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL4Position, WristConstants.kWristL4Position, 
    //     ExecutionOrder.ALL_TOGETHER, 10.0);

    //     return superSystemCommand;
    // }

    // public Command moveTogroundIntake() {
    //     SuperSystemCommand superSystemCommand = new SuperSystemCommand(pivot, elevator, wrist, V1ElevatorConstants.kElevatorPivotGroundIntake, ElevatorConstants.kElevatorGroundIntake, WristConstants.kWristGroundIntake, ExecutionOrder.ELV_WRT_PVT, 10);
    //     return superSystemCommand;
    return Commands.sequence(
        Commands.runOnce(() -> isStarted = false),
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL4Position,WristConstants.kIntermediatePosition,
        () -> pivot.atPosition(), () -> elevator.atPosition(), () -> wrist.atPosition()),

        wrist.setPositionCommand(WristConstants.kWristL4Position)
    );
    }

    public void initialize() {
        pivot.setEnabled(true);
        wrist.setEnabled(true);
        elevator.setEnabled(true);
        pivot.setTargetPosition(0);
        elevator.setPosition(0);
        wrist.setPosition(0);
        ammountCalled = 0;
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
        double pivotAngle, double elevatorPosition, double wristAngle,
        BooleanSupplier pivotAtPosition, BooleanSupplier elevatorAtPosition, BooleanSupplier wristAtPosition
    ) {
        return Commands.run(() -> {
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
        })
        .until(
            () -> ((pivot.atPosition() && elevator.atPosition() && wrist.atPosition()) || (Timer.getFPGATimestamp() - startTime >= timeout))
        )
        ;
    }

    // public void execute(BooleanSupplier pivotAtPosition, BooleanSupplier elevatorAtPosition, BooleanSupplier wristAtPosition)
    // {
    //     if(!isStarted) {
    //         isStarted = true;
    //         startTime = Timer.getFPGATimestamp();
    //     }

    //     updateDependencies(); 

    //     switch (exeOrder) {
    //         case ALL_TOGETHER:
    //             pivot.setEnabled(true);
    //             wrist.setEnabled(true);
    //             elevator.setEnabled(true);
    //             break;

    //         case ELV_PVT_WRT:
    //             elevator.setEnabled(true);
    //             if(elevator.atPosition()) {
    //                 pivot.setEnabled(true);
    //             }
    //             if(pivot.atPosition()) {
    //                 wrist.setEnabled(true);
    //             }
    //             break;

    //         case ELV_WRT_PVT:
    //             elevator.setEnabled(true);
    //             if(elevator.atPosition()) {
    //                 wrist.setEnabled(true);
    //             }
    //             if(wrist.atPosition()) {
    //                 pivot.setEnabled(true);
    //             }
    //             break;

    //         case PVT_WRT_ELV:
    //             pivot.setEnabled(true);
    //             if(pivot.atPosition()) {
    //                 wrist.setEnabled(true);
    //             }
    //             if(wrist.atPosition()) {
    //                 elevator.setEnabled(true);
    //             }
    //             break;

    //         case PVT_ELV_WRT:
    //             pivot.setEnabled(true);
    //             if(pivot.atPosition()) {
    //                 elevator.setEnabled(true);
    //             }
    //             if(elevator.atPosition()) {
    //                 wrist.setEnabled(true);
    //             }
    //             break;

    //         case WRT_ELV_PVT:
    //             wrist.setEnabled(true);
    //             if(wrist.atPosition()) {
    //                 elevator.setEnabled(true);
    //             }
    //             if(elevator.atPosition()) {
    //                 pivot.setEnabled(true);
    //             }
    //             break;

    //         case WRT_PVT_ELV:
    //             ammountCalled += 1;
    //             SmartDashboard.putNumber("Ammount Called", ammountCalled);
    //             wrist.setEnabled(true);
    //             SmartDashboard.putBoolean("Wrist at Position", false);
    //             if(wristAtPosition.getAsBoolean()) {
    //                 SmartDashboard.putBoolean("Wrist at Position", true);
    //                 pivot.setEnabled(true);
    //             }

    //             SmartDashboard.putBoolean("Pivot at Position", false);
    //             if(pivotAtPosition.getAsBoolean()) {
    //                 SmartDashboard.putBoolean("Pivot at position", false);
    //                 elevator.setEnabled(true);
    //             }
    //             break;
        
    //         default:
    //             break;
    //     }
    // }
}
