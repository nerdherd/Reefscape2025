package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class PreloadTaxi extends SequentialCommandGroup{
    public PreloadTaxi(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException{

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.sequence(
                Commands.sequence(
                    superSystem.holdPiece(),
                    Commands.waitSeconds(3),
                    AutoBuilder.followPath(pathGroup.get(0)), 
                    superSystem.moveToAuto(PositionEquivalents.L4Auto)
                ),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    superSystem.outtake(),
                    Commands.waitSeconds(1),
                    superSystem.stopRoller(),
                    superSystem.moveTo(PositionEquivalents.L1),
                    AutoBuilder.followPath(pathGroup.get(1)), 
                    superSystem.moveTo(PositionEquivalents.Stow)
                )
        )
        );
    }
}
