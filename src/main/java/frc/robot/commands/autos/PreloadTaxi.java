package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import frc.robot.Constants.SuperSystemConstants.NamedPositions;
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

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            // Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.sequence(
                Commands.sequence(
                    superSystem.holdPiece(),
                    AutoBuilder.followPath(pathGroup.get(0)), 
                    superSystem.moveToAuto(NamedPositions.L4)
                ),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    superSystem.outtake(),
                    Commands.waitSeconds(1),
                    superSystem.stopRoller(),
                    superSystem.moveTo(NamedPositions.L5),
                    AutoBuilder.followPath(pathGroup.get(1)), 
                    superSystem.moveTo(NamedPositions.Stow)
                )
        )
        );
    }
}
