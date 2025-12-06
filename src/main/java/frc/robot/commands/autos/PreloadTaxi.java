package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

public class PreloadTaxi extends SequentialCommandGroup{
    public PreloadTaxi(String autoname, SuperSystem superSystem, SwerveDrivetrain swerve) throws IOException, ParseException{

        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            // Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            // Commands.runOnce(swerve.getImu()::zeroAll),
            
            Commands.sequence(
                AutoBuilder.followPath(pathGroup.get(0)),
                Commands.waitSeconds(0.5),
                superSystem.moveToAuto(PositionEquivalents.SemiStow),
                superSystem.moveToAuto(PositionEquivalents.L4),
                Commands.waitSeconds(1),

                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                Commands.waitSeconds(1),

                superSystem.moveToAuto(PositionEquivalents.L2),
                superSystem.moveToAuto(PositionEquivalents.SemiStow)
            )
        );
    }
}
