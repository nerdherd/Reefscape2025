package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;

public class Taxi extends SequentialCommandGroup{
    public Taxi(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException{

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);

        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        addCommands(
            swerve.setVisionEnabledCommand(false),
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.sequence(
                
                AutoBuilder.followPath(pathGroup.get(0))

            )
        );
    }
}
