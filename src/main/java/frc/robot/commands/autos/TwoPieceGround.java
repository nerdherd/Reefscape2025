package frc.robot.commands.autos;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPieceGround extends SequentialCommandGroup {
    public TwoPieceGround(String autoname, SuperSystem superSystem, SwerveDrivetrain swerve) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();
        
        addCommands(
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            
            Commands.sequence(
                // Move to Reef
                AutoBuilder.followPath(pathGroup.get(0)),
                superSystem.moveToAuto(PositionEquivalents.L4),
                Commands.waitSeconds(0.5),

                // Outtake
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveToAuto(PositionEquivalents.SemiStow),

                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(1)),
                    superSystem.moveToAuto(PositionEquivalents.GroundIntake)
                ),
                Commands.waitSeconds(0.1),

                // Move to and intake ground coral
                Commands.parallel(
                    superSystem.intakeUntilSensed(2.5),
                    AutoBuilder.followPath(pathGroup.get(2))
                ),
                superSystem.stopRoller(),
                
                // Move to Reef
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(3)),
                    superSystem.moveTo(PositionEquivalents.SemiStow)
                ),
                superSystem.moveToAuto(PositionEquivalents.L4),
                Commands.waitSeconds(0.5),

                // Outtake
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),

                superSystem.moveToAuto(PositionEquivalents.SemiStow)
            )
        );
    }
}
