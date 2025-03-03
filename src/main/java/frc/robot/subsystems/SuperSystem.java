package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RollerConstants;
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

        initialize(); // todo need to move to each mode's init in container 
        //(after power on, during Disable mode, motors disabled and not applying brake)
        //(after auto/teleop mode, During disable mode, motor disabled and applying brake)
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
        return claw.setClawPositionCommand(ClawConstants.kClosedPosition);
    }

    public Command intakeCoral() {
        return Commands.sequence(
            claw.setClawPositionCommand(ClawConstants.kCoralReleasePosition),
            claw.setVelocityCommand(RollerConstants.kIntakePower),
            Commands.waitSeconds(2), //taking coral
            claw.setClawPositionCommand(ClawConstants.kCoralHoldPosition),
            Commands.waitSeconds(0.1),
            claw.setVelocityCommand(-0.01) // holding coral
        );
    }

    public Command outtakeCoral() {
        return Commands.sequence(
            claw.setVelocityCommand(RollerConstants.kOuttakePower),
            Commands.waitSeconds(0.5),
            claw.setClawPositionCommand(ClawConstants.kCoralReleasePosition),
            Commands.waitSeconds(0.5),
            claw.setVelocityCommand(0)
        );
    }

    public Command intakeAlgae() {
        return Commands.sequence(
            claw.setClawPositionCommand(ClawConstants.kAlgaeReleasePosition),
            claw.setVelocityCommand(RollerConstants.kIntakePower),
            Commands.waitSeconds(2), // taking algae
            claw.setClawPositionCommand(ClawConstants.kAlgaeHoldPosition),
            Commands.waitSeconds(0.1),
            claw.setVelocityCommand(-0.01) // holding algae
        );
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
            claw.setVelocityCommand(RollerConstants.kOuttakePower),
            Commands.waitSeconds(0.5),
            claw.setClawPositionCommand(ClawConstants.kAlgaeReleasePosition),
            Commands.waitSeconds(0.5),
            claw.setVelocityCommand(0)
        );
    }

    public Command moveTo(NamedPositions position) {
        return Commands.sequence(
            preExecute(),
            execute(position.executionOrder, 10.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            
            wrist.setPositionCommand(position.finalWristPosition)
        );
    }

    public Command moveToStow() {
        return Commands.sequence(
            preExecute(),
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kStowPosition)
        );
    }

    public Command moveToSemiStow() {
        return Commands.sequence(
            preExecute(),
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotSemiStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition)
        );
    }

    public Command moveToCage() { //TODO
        return moveTo(NamedPositions.Cage);
    }

    public Command moveToNet() { //TODO
        return moveTo(NamedPositions.Net);
    }

    public Command moveToProcessor() { //TODO
        return moveTo(NamedPositions.Processor);
    }    

    public Command moveTogroundIntake() {
        return Commands.sequence(
            preExecute(),
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
            preExecute(),
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotStationPosition, ElevatorConstants.kElevatorStationPosition, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kStationPosition)/* ,
            intakeCoral(),
            Commands.race(
                Commands.waitSeconds(5),
                stopRoller()
            )*/
        );
    }

    public Command moveToGroundIntake()
    {
        return Commands.none(); //todo
    }

    public Command moveToL3L4()
    {
        return Commands.none(); //todo
    }

    public Command moveToL2L3()
    {
        return Commands.none();//todo
    }

    public Command moveToL1() {
        return Commands.sequence(
            preExecute(),
            execute(ExecutionOrder.WRT_PVT_ELV,10.0,
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position,WristConstants.kIntermediatePosition),

            wrist.setPositionCommand(WristConstants.kWristL1Position)
        );

    }

    public Command moveToL2() {
        return Commands.sequence(
            preExecute(),
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kWristL2Position)
        );
    }

    public Command moveToL3() {
    return Commands.sequence(
        preExecute(),
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position,WristConstants.kIntermediatePosition),

        wrist.setPositionCommand(WristConstants.kWristL3Position)
    );
    }

    public Command moveToL4() { // todo: need to measure the height before to run
    return Commands.sequence(
        preExecute(),
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
        pivot.setTargetPosition(0.0);
        elevator.setTargetPosition(0.0);
        wrist.setTargetPosition(0.0);
        claw.setClawPosition(ClawConstants.kClosedPosition);
        claw.setVelocityCommand(0.0);
        isStarted = false;
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPosition();
        pivot.setElevatorLength(elevator.getPosition()); 
        elevator.setPivotAngle(curPivotAngle);
        wrist.setPivotAngle(curPivotAngle);
    }

    public Command preExecute()
    {
        return Commands.runOnce(()-> {
            pivot.stopMotion();
            elevator.stopMotion();
            wrist.stopMotion();
            pivot.setTargetPosition(pivot.getPosition());
            elevator.setTargetPosition(elevator.getPosition());
            wrist.setTargetPosition(wrist.getPosition());
        }, pivot, elevator, wrist);
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
            
            // TODO move this functionality to each subsystem
            updateDependencies(); 
    
            switch (exeOrder) {
                case ALL_TOGETHER:
                    pivot.setTargetPosition(pivotAngle);
                    elevator.setTargetPosition(elevatorPosition);
                    wrist.setTargetPosition(wristAngle);
                    break;
    
                case ELV_PVT_WRT:
                    elevator.setTargetPosition(elevatorPosition);
                    if(elevatorAtPosition.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        if(pivotAtPosition.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                        }
                    }
                    break;
    
                case ELV_WRT_PVT:
                    elevator.setTargetPosition(elevatorPosition);
                    if(elevatorAtPosition.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        if(wristAtPosition.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                        }
                    }
                    break;
    
                case PVT_WRT_ELV:
                    pivot.setTargetPosition(pivotAngle);
                    if(pivotAtPosition.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        if(wristAtPosition.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
                        }
                    }
                    break;
    
                case PVT_ELV_WRT:
                    pivot.setTargetPosition(pivotAngle);
                    if(pivotAtPosition.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        if(elevatorAtPosition.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                        }
                    }
                    break;
    
                case WRT_ELV_PVT:
                    wrist.setTargetPosition(wristAngle);
                    if(wristAtPosition.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        if(elevatorAtPosition.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                        }
                    }
                    break;
    
                case WRT_PVT_ELV:
                    wrist.setTargetPosition(wristAngle);
                    if(wristAtPosition.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        if(pivotAtPosition.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
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
