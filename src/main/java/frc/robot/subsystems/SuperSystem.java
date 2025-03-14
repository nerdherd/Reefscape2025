package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
public class SuperSystem {
    public Elevator elevator;
    public ElevatorPivot pivot;
    public IntakeWrist wrist;
    public IntakeRoller intakeRoller;
    public Climb climbMotor;

    public StatusSignal<S1StateValue> intakeSensor;
    public StatusSignal<S2StateValue> floorSensor;
    
    public NamedPositions currentPosition = NamedPositions.Stow;
    public NamedPositions lastPosition;
    
    boolean elevatorWithinRange;

    private BooleanSupplier pivotAtPosition, elevatorAtPosition, wristAtPosition,pivotAtPositionWide, elevatorAtPositionWide, wristAtPositionWide, intakeDetected, floorDetected;

    public enum ExecutionOrder {
        ALL_TOGETHER,
        ELV_PVT_WRT,
        ELV_WRT_PVT,
        PVT_ELV_WRT,
        PVT_WRT_ELV,
        WRT_ELV_PVT,
        WRT_PVT_ELV,
        WRTELV_PVT
    }

    private boolean isStarted = false;
    private boolean wristSet = false, elevatorSet = false, pivotSet = false;
    private double startTime = 0;

    public SuperSystem(Elevator elevator, ElevatorPivot pivot, IntakeWrist wrist, IntakeRoller intakeRoller, CANdi candi, Climb climbMotor) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.intakeRoller = intakeRoller;
        this.intakeSensor = candi.getS1State(true);
        this.floorSensor = candi.getS2State(true);
        this.climbMotor = climbMotor;

        pivotAtPosition = () -> pivot.atPosition();
        pivotAtPositionWide = () -> pivot.atPositionWide();
        elevatorAtPosition = () -> elevator.atPosition();
        elevatorAtPositionWide = () -> elevator.atPositionWide();
        wristAtPosition = () -> wrist.atPosition();
        wristAtPositionWide = () -> wrist.atPositionWide();
        intakeDetected = () -> (intakeSensor.getValue().value == 1);
        floorDetected = () -> (floorSensor.getValue().value == 1);
        

        ShuffleboardTab tab = Shuffleboard.getTab("Supersystem");
        tab.addBoolean("isStarted", () -> isStarted);
        initialize(); // todo need to move to each mode's init in container 
        //(after power on, during Disable mode, motors disabled and not applying brake)
        //(after auto/teleop mode, During disable mode, motor disabled and applying brake)
    }

    // subsystems
    public void reConfigureMotors() {
        pivot.configureMotorV1();
        elevator.setMotorConfigs();
        wrist.configurePID(wrist.motorConfigs);
        intakeRoller.configureMotor(intakeRoller.motorConfigs);

    }

    public Command zeroEncoders() {
        return Commands.runOnce(()-> {
            pivot.zeroEncoder();
            elevator.zeroEncoder();
            wrist.zeroEncoder();
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            pivot.stop();
            wrist.stopCommand();
            elevator.stopCommand();
        });
    }

    public Command stopRoller() {
        return intakeRoller.setVoltageCommand(0.0);
    }


    public Command intake() {
        return intakeRoller.setVoltageCommand(RollerConstants.kIntakePower);
    }

    public Command repositionCoralLeft() {
        return Commands.sequence(
            intakeRoller.setVoltageCommandLeft(1),
            intakeRoller.setVoltageCommandRight(2.0)

        );
    }

    public Command repositionCoralRight() {
        return Commands.sequence(
            intakeRoller.setVoltageCommandLeft(-2.0),
            intakeRoller.setVoltageCommandRight(-1)

        );
    }

    public Command intakeUntilSensed() {
        return Commands.sequence(
            intake(), 
            Commands.race(Commands.waitUntil(
                intakeDetected),
                Commands.waitSeconds(5)),
            holdPiece()
        );
    }

    public Command intakeUntilSensed(double timeout) {
        return Commands.sequence(
            intake(), 
            Commands.race(Commands.waitUntil(
                intakeDetected),
                Commands.waitSeconds(timeout)),
            holdPiece()
        );
    }

    public Command holdPiece() {
        return intakeRoller.setVoltageCommand(-1); // holding coral
    }

    public Command outtake() {
        if (currentPosition == NamedPositions.L1) { // TODO is it working??
            return intakeRoller.setVoltageCommandLeft(RollerConstants.kL1OuttakePower); // Might need to make new constant for this
        }
        else {
            return intakeRoller.setVoltageCommand(RollerConstants.kOuttakePower);
        }
    }

    public Command shootAlgae() {
        return intakeRoller.setVoltageCommand(4.25);
    } 
    
    public Command climbPrep() {
        return climbMotor.setVoltageCommand(0.5);
    }

    public Command climbHardClamp() {
        return climbMotor.setVoltageCommand(-3.0);
    }

    public Command climbSoftClamp() {
        return climbMotor.setVoltageCommand(-0.4);
    }

    public Command stopClimb() {
        return climbMotor.setVoltageCommand(0.0);
    }

    public Command climbCommandUp() {
        return Commands.sequence(
            climbPrep(), 
            moveTo(NamedPositions.ClimbUp) 
        );
    }

    
    public Command climbCommandDown() {
        return Commands.sequence(
            climbHardClamp(), 
            moveTo(NamedPositions.ClimbDown) 
        );
    }

    public Command updatePositions(NamedPositions position) {
        return Commands.runOnce(() -> {
            lastPosition = currentPosition;
            currentPosition = position;
        });
    }

    public Command moveTo(NamedPositions position) {
        return Commands.sequence(
            updatePositions(position),
            // Commands.waitSeconds(.001),
            goTo(position)
        );
    }

    // movement
    private Command goTo(NamedPositions position) {
        if (position == NamedPositions.GroundIntake || lastPosition == NamedPositions.GroundIntake) {
            return Commands.sequence(
                preExecute(),
                execute(NamedPositions.intermediateGround.executionOrder, 10.0, 
                NamedPositions.intermediateGround.pivotPosition, NamedPositions.intermediateGround.elevatorPosition, NamedPositions.intermediateGround.intermediateWristPosition),
                wrist.setPositionCommand(NamedPositions.intermediateGround.finalWristPosition),
                preExecute(),
                execute(position.executionOrder, 10.0, 
                position.pivotPosition, position.elevatorPosition, position.finalWristPosition).until(floorDetected)
            );
        }
        if (position.intermediateWristPosition == position.finalWristPosition)
            return Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 10.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition)
                              
            );
        return Commands.sequence(
            preExecute(),
            execute(position.executionOrder, 10.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            wrist.setPositionCommand(position.finalWristPosition)
            
        );
    }

    public Command moveToAuto(NamedPositions position) {
        // currentPosition = position;
        if (position.intermediateWristPosition == position.finalWristPosition)
            return Commands.sequence(
                preExecute(),
                execute(position.executionOrder, 5.0, 
                position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition)
                              
            );
        else return Commands.sequence(
            preExecute(),
            execute(position.executionOrder, 5.0, 
            position.pivotPosition, position.elevatorPosition, position.intermediateWristPosition),
            wrist.setPositionCommand(position.finalWristPosition)
            
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

    public void initialize() {
        pivot.setEnabled(true);
        wrist.setEnabled(true);
        elevator.setEnabled(true);
        intakeRoller.setEnabled(true);
        
        pivot.setTargetPosition(0.0);
        elevator.setTargetPosition(0.0);
        wrist.setTargetPosition(0.0);
        intakeRoller.setVoltageCommand(0.0);
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

            if (pivotAngle == PivotConstants.kElevatorPivotStowPosition) {
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

                case WRTELV_PVT:
                wrist.setTargetPosition(wristAngle);
                wristSet = true;
                elevator.setTargetPosition(elevatorPosition);
                elevatorSet = true;
                if (wristAtPositionWide.getAsBoolean() && elevatorAtPositionWide.getAsBoolean()) {
                    pivot.setTargetPosition(pivotAngle);
                    pivotSet = true;
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

    public void initShuffleboard(LOG_LEVEL priority){
        if (priority == LOG_LEVEL.OFF) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Supersystem");
        switch (priority) {
            case OFF:
                break;
            case ALL:
                tab.addString("Super System Current Position", () -> currentPosition.toString());
            case MEDIUM:
            case MINIMAL:
                break;
        }
    }

}
