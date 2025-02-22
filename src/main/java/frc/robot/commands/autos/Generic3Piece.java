package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Generic3Piece extends SequentialCommandGroup {
    private IntakeRoller intakeRoller;
    private Elevator elevator;
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;

    public Generic3Piece(SwerveDrivetrain swerve, IntakeRoller intakeRoller, Elevator elevator, String autoPath) 
    throws IOException, ParseException {
        this.intakeRoller = intakeRoller;
        this.elevator = elevator;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto("L3", "L3", "L3")
        );
    }

    public Command runAuto(String pos1, String pos2, String pos3) {
        return Commands.sequence(
            // Place preload
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos1) {
                case "L1": elevator.moveToReefL1(); break;
                case "L2": elevator.moveToReefL2(); break;
                case "L3": elevator.moveToReefL3(); break;
                case "L4": elevator.moveToReefL4(); break;
                }
            }),
            intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            intakeRoller.stop(),

            // Drive to Coral Station and intake coral 2
            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            elevator.moveToStation(),
            intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            intakeRoller.stop(),

            // Drive to Reef and place coral 2
            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            Commands.runOnce(() -> {
                switch (pos2) {
                case "L1": elevator.moveToReefL1(); break;
                case "L2": elevator.moveToReefL2(); break;
                case "L3": elevator.moveToReefL3(); break;
                case "L4": elevator.moveToReefL4(); break;
                }
            }),
            intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            intakeRoller.stop(),
            
            // Drive to Coral Station and intake coral 3
            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(3))
            ),
            elevator.moveToStation(),
            intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            intakeRoller.stop(),

            // Drive to Reef and place coral 3
            Commands.parallel(
                elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(4))
            ),
            Commands.runOnce(() -> {
                switch (pos3) {
                case "L1": elevator.moveToReefL1(); break;
                case "L2": elevator.moveToReefL2(); break;
                case "L3": elevator.moveToReefL3(); break;
                case "L4": elevator.moveToReefL4(); break;
                }
            }),
            intakeRoller.outtake(),
            Commands.waitSeconds(1.5),
            stopAuto()
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            intakeRoller.stop(),
            elevator.stow()
        );
    }
}
