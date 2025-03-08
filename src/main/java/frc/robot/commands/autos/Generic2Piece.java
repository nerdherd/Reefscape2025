package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import frc.robot.Constants.SuperSystemConstants.NamedPositions;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Generic2Piece extends SequentialCommandGroup {
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;
    public SuperSystem superSystem;

    public Generic2Piece(SwerveDrivetrain swerve, SuperSystem superSystem, String autoPath) 
    throws IOException, ParseException {
        this.superSystem = superSystem;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto("L2", "L2")
        );
    }


    public Command runAuto(String pos1, String pos2) {
        return Commands.sequence(
            // Place preload
            AutoBuilder.followPath(pathGroup.get(0)),
            superSystem.moveTo(NamedPositions.L2),
            Commands.runOnce(() -> {
                switch (pos1) {
                case "L1": superSystem.moveTo(NamedPositions.L1);
                case "L2": superSystem.moveTo(NamedPositions.L2);
                case "L3": superSystem.moveTo(NamedPositions.L3);
                case "L4": superSystem.moveTo(NamedPositions.L4);
                }
            }),
            superSystem.outtake(),
            Commands.waitSeconds(1.5),
            superSystem.intakeRoller.stop(),

            // Drive to Coral Station and intake coral 2
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
            Commands.runOnce(() -> {
                switch (pos2) {
                case "L1": superSystem.moveTo(NamedPositions.L1);
                case "L2": superSystem.moveTo(NamedPositions.L2);
                case "L3": superSystem.moveTo(NamedPositions.L3);
                case "L4": superSystem.moveTo(NamedPositions.L4);
                }
            }),
            superSystem.intakeRoller.outtake(),
            Commands.waitSeconds(1.5),

            // Stop
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
