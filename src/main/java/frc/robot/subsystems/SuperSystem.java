package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    public NamedPositions currentPosition = NamedPositions.Stow;
    
    boolean elevatorWithinRange;

    private BooleanSupplier pivotAtPosition, elevatorAtPosition, wristAtPosition,pivotAtPositionWide, elevatorAtPositionWide, wristAtPositionWide;

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
    private boolean wristSet = false, elevatorSet = false, pivotSet = false;
    private double startTime = 0;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, IntakeV2 claw) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.claw = claw;

        pivotAtPosition = () -> pivot.atPosition();
        pivotAtPositionWide = () -> pivot.atPositionWide();
        elevatorAtPosition = () -> elevator.atPosition();
        elevatorAtPositionWide = () -> elevator.atPositionWide();
        wristAtPosition = () -> wrist.atPosition();
        wristAtPositionWide = () -> wrist.atPositionWide();

        ShuffleboardTab tab = Shuffleboard.getTab("Supersystem");
        tab.addBoolean("isStarted", () -> isStarted);
        initialize(); // todo need to move to each mode's init in container 
        //(after power on, during Disable mode, motors disabled and not applying brake)
        //(after auto/teleop mode, During disable mode, motor disabled and applying brake)
    }

    // subsystems
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
        return claw.setVoltageCommand(0.0);
    }

    public Command closeClaw() {
        return Commands.sequence(
            claw.setClawPositionCommand(ClawConstants.kClosedPosition),
            claw.setVoltageCommand(0)
        );
    }

    public Command moveClawUp() {
        return Commands.runOnce(
            () -> wrist.incrementOffset(0.001)
        );
    }

    public Command moveClawDown() {
        return Commands.runOnce(
            () -> wrist.incrementOffset(-0.001)
        );
    }

    // coral
    public Command intakeCoral() {
        return Commands.sequence(
            claw.setClawPositionCommand(
                currentPosition == NamedPositions.GroundIntake ? ClawConstants.kCoralOpenPosition : ClawConstants.kStationPosition
            ),
            claw.setVoltageCommand(RollerConstants.kIntakePower)
        );
    }

    public Command holdCoral() {
        return Commands.sequence(
            claw.setClawPositionCommand(
                ClawConstants.kCoralHoldPosition
            ),
            claw.setVoltageCommand(-2) // holding coral
        );
    }

    public Command outtakeCoral() {
        return Commands.sequence(
            claw.setVoltageCommand(RollerConstants.kOuttakePower),
            Commands.waitSeconds(0.5),
            claw.setClawPositionCommand(ClawConstants.kCoralReleasePosition)
        );
    }

    // algae
    public Command intakeAlgae() {
        return Commands.sequence(
            claw.setClawPositionCommand(ClawConstants.kAlgaeOpenPosition),
            claw.setVoltageCommand(-4.25)
        );
    }

    public Command holdAlgae() {
        return Commands.sequence(
            claw.setClawPositionCommand(ClawConstants.kAlgaeHoldPosition),
            claw.setVoltageCommand(-2)
        );
    }

    public Command outtakeAlgae() {
        return Commands.sequence(
            claw.setVoltageCommand(4.25),
            Commands.waitSeconds(0.5),
            claw.setClawPositionCommand(ClawConstants.kAlgaeReleasePosition)
        );
    }

    // movement
    public Command moveTo(NamedPositions position) {
        // currentPosition = position;
        if (position.intermediateWristPosition == position.finalWristPosition)
            return Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 10.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition)
                              
            );
        else return Commands.sequence(
            preExecute(),
            execute(position.executionOrder, 10.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            wrist.setPositionCommand(position.finalWristPosition)
            
        );
    }

    public Command moveToStow() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition),
            wrist.setPositionCommand(WristConstants.kStowPosition)

        );
    }

    public Command moveToSemiStow() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.WRT_ELV_PVT, 10.0, 
            V1ElevatorConstants.kElevatorPivotSemiStowPosition, ElevatorConstants.kElevatorStowPosition, WristConstants.kIntermediatePosition)
        );
    }

    // game elements
    public Command moveToCage() { //TODO
        return moveTo(NamedPositions.Cage);
    }

    public Command moveToNet() { //TODO
        return moveTo(NamedPositions.Net);
    }

    public Command moveToProcessor() { //TODO
        // note: we may not need this one, because the intake action could cover it.
        return moveTo(NamedPositions.Processor);
    }    

    public Command moveToGroundIntake() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.PVT_WRT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotStowPosition, ElevatorConstants.kElevatorGroundIntake, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kWristGroundIntake)
        );
    }

    public Command moveToStation() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
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

    public Command moveToL3L4() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.PVT_WRT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3L4Position, WristConstants.kL234AlagePosition),
            
            wrist.setPositionCommand(WristConstants.kL234AlagePosition)/* ,
            //todo
            intakeAlage(),
            )*/
        );
    }

    public Command moveToL2L3()
    {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.PVT_WRT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2L3Position, WristConstants.kL234AlagePosition),
            
            wrist.setPositionCommand(WristConstants.kL234AlagePosition)/* ,
            //todo            
            intakeAlage(),
            )*/
        );
    }

    public Command moveToL1() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.WRT_PVT_ELV,10.0,
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL1Position,WristConstants.kIntermediatePosition),

            wrist.setPositionCommand(WristConstants.kWristL1Position)
        );

    }

    public Command moveToL2() {
        return Commands.sequence(
            preExecute(),
            //wrist.setPositionCommand(WristConstants.), //TODO pre-position
            execute(ExecutionOrder.WRT_PVT_ELV, 10.0, 
            V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL2Position, WristConstants.kIntermediatePosition),
            
            wrist.setPositionCommand(WristConstants.kWristL2Position)
        );
    }

    public Command moveToL3() {
    return Commands.sequence(
        preExecute(),
        //wrist.setPositionCommand(WristConstants.), //TODO pre-position
        execute(ExecutionOrder.WRT_PVT_ELV,10.0,
        V1ElevatorConstants.kElevatorPivotPositionVertical, ElevatorConstants.kElevatorL3Position,WristConstants.kIntermediatePosition),

        wrist.setPositionCommand(WristConstants.kWristL3Position)
    );
    }

    public Command moveToL4() { // todo: need to measure the height before to run
    return Commands.sequence(
        preExecute(),
        //wrist.setPositionCommand(WristConstants.), //TODO pre-position
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
        claw.setVoltageCommand(0.0);
        isStarted = false;
    }

    public void updateDependencies() { 
        double curPivotAngle = pivot.getPosition();
        // pivot.setElevatorLength(elevator.getPosition()); // TODO fake
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

    public Command execute(ExecutionOrder exeOrder, double timeout, 
                           double pivotAngle, double elevatorPosition, double wristAngle)
    {
        return Commands.runEnd(() -> {
            if (!isStarted) {
                isStarted = true;
                startTime = Timer.getFPGATimestamp();
                wristSet = false;
                pivotSet = false;
                elevatorSet = false;
            }

            if (pivotAngle == V1ElevatorConstants.kElevatorPivotStowPosition) {
                elevatorWithinRange = elevator.atPositionWide();
            } else {
                elevatorWithinRange = elevator.atPosition();
            }
            
            // TODO move this functionality to each subsystem
            updateDependencies(); 
    
            switch (exeOrder) {
                case ALL_TOGETHER:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    elevator.setTargetPosition(elevatorPosition);
                    wristSet = true;
                    wrist.setTargetPosition(wristAngle);
                    elevatorSet = true;
                    break;
    
                case ELV_PVT_WRT:
                    elevator.setTargetPosition(elevatorPosition);
                    elevatorSet = true;
                    if (elevatorAtPositionWide.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        pivotSet = true;
                        if (pivotAtPositionWide.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                            wristSet = true;
                        }
                    }
                    break;
    
                case ELV_WRT_PVT:
                    elevator.setTargetPosition(elevatorPosition);
                    elevatorSet = true;
                    if (elevatorAtPositionWide.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        wristSet = true;
                        if (wristAtPositionWide.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                            pivotSet = true;
                        }
                    }
                    break;
    
                case PVT_WRT_ELV:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    if (pivotAtPositionWide.getAsBoolean()) {
                        wrist.setTargetPosition(wristAngle);
                        wristSet = true;
                        if (wristAtPositionWide.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
                            elevatorSet = true;
                        }
                    }
                    break;
    
                case PVT_ELV_WRT:
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
                    if (pivotAtPositionWide.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                        if (elevatorAtPositionWide.getAsBoolean()) {
                            wrist.setTargetPosition(wristAngle);
                            wristSet = true;
                        }
                    }
                    break;
    
                case WRT_ELV_PVT:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    if (wristAtPositionWide.getAsBoolean()) {
                        elevator.setTargetPosition(elevatorPosition);
                        elevatorSet = true;
                        if (elevatorAtPositionWide.getAsBoolean()) {
                            pivot.setTargetPosition(pivotAngle);
                            pivotSet = true;
                        }
                    }
                    break;
    
                case WRT_PVT_ELV:
                    wrist.setTargetPosition(wristAngle);
                    wristSet = true;
                    if (wristAtPositionWide.getAsBoolean()) {
                        pivot.setTargetPosition(pivotAngle);
                        pivotSet = true;
                        if (pivotAtPositionWide.getAsBoolean()) {
                            elevator.setTargetPosition(elevatorPosition);
                            elevatorSet = true;
                        }
                    }
                    break;
            
                default:
                    break;
            }
        },
        () -> {
            isStarted = false;
            wristSet = false;
            pivotSet = false;
            elevatorSet = false;
        }
        ).until(
            () -> (
                (pivot.atPosition() && pivotSet
                && elevatorWithinRange && elevatorSet
                && wrist.atPosition() && wristSet)
                || (Timer.getFPGATimestamp() - startTime >= timeout)
            )
        );
    }

}
