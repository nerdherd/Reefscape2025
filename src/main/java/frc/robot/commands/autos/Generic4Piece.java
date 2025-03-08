package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.subsystems.SuperSystem;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Generic4Piece extends SequentialCommandGroup {
    private SuperSystem superSystem;
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;

    public Generic4Piece(SwerveDrivetrain swerve, SuperSystem superSystem, String autoPath) 
    throws IOException, ParseException {
        this.superSystem = superSystem;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto("L3", "L3", "L3", "L3")
        );
    }

    public Command runAuto(String pos1, String pos2, String pos3, String pos4) {
        return Commands.sequence(
            // Place preload
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos1) {
                case "L1": superSystem.moveTo(NamedPositions.L1); break;
                case "L2": superSystem.moveTo(NamedPositions.L2); break;
                case "L3": superSystem.moveTo(NamedPositions.L3); break;
                case "L4": superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.intakeRoller.stop(),

            // Drive to Coral Station and intake coral 2%
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(1))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            superSystem.intakeRoller.stop(),

            // Drive to Reef and place coral 2
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(2))
            ),
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos2) {
                case "L1": superSystem.moveTo(NamedPositions.L1); break;
                case "L2": superSystem.moveTo(NamedPositions.L2); break;
                case "L3": superSystem.moveTo(NamedPositions.L3); break;
                case "L4": superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.intakeRoller.stop(),
            
            // Drive to Coral Station and intake coral 3
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(3))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            superSystem.intakeRoller.stop(),

            // Drive to Reef and place coral 3
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(4))
            ),
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos3) {
                case "L1": superSystem.moveTo(NamedPositions.L1); break;
                case "L2": superSystem.moveTo(NamedPositions.L2); break;
                case "L3": superSystem.moveTo(NamedPositions.L3); break;
                case "L4": superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.intakeRoller.stop(),
            
            // Drive to Coral Station and intake coral 4
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(5))
            ),
            // elevator.moveToStation(),
            // intakeRoller.intake(),
            Commands.waitSeconds(2.5),
            superSystem.intakeRoller.stop(),

            // Drive to Reef and place coral 4
            Commands.parallel(
                // elevator.stow(),
                AutoBuilder.followPath(pathGroup.get(6))
            ),
            AutoBuilder.followPath(pathGroup.get(0)),
            Commands.runOnce(() -> {
                switch (pos1) {
                case "L1": superSystem.moveTo(NamedPositions.L1); break;
                case "L2": superSystem.moveTo(NamedPositions.L2); break;
                case "L3": superSystem.moveTo(NamedPositions.L3); break;
                case "L4": superSystem.moveTo(NamedPositions.L4); break;
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            stopAuto()
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            superSystem.intakeRoller.stop()//,
            // elevator.stow()
        );
    }
}
