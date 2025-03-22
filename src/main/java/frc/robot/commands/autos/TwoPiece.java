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


public class TwoPiece extends SequentialCommandGroup {
    public TwoPiece(SwerveDrivetrain swerve, String autoname, SuperSystem superSystem) throws IOException, ParseException {
        
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
                    // superSystem.holdPiece(),
                    AutoBuilder.followPath(pathGroup.get(0)),
                    Commands.sequence(
                        // superSystem.moveToAuto(PositionEquivalents.L4)
                        // superSystem.moveToAuto(PositionEquivalents.L1)
                    )
                ),
                Commands.sequence(
                    // superSystem.outtake(),
                    Commands.waitSeconds(2.0)
                    // superSystem.stopRoller()
                ),
                Commands.sequence(
                    // superSystem.moveTo(PositionEquivalents.L5),
                    // superSystem.moveTo(PositionEquivalents.L1),
                    Commands.parallel(
                        AutoBuilder.followPath(pathGroup.get(1)),
                        // superSystem.moveToAuto(PositionEquivalents.GroundIntake),
                        // superSystem.moveToAuto(PositionEquivalents.Stow),
                        Commands.waitSeconds(2.0)
                    )
                    
                ),
                Commands.sequence(
                    // superSystem.intake(),
                    // superSystem.intakeUntilSensed(2),
                    Commands.waitSeconds(2)
                    // superSystem.holdPiece()
                ),
                Commands.sequence(
                    Commands.parallel(
                        Commands.sequence(
                            Commands.waitSeconds(0.3)
                        ),
                        AutoBuilder.followPath(pathGroup.get(2))
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(1)
                        // superSystem.moveToAuto(PositionEquivalents.L1)
                    )
                ),
                Commands.sequence(
                    // superSystem.outtake(),
                    Commands.waitSeconds(2)
                    // superSystem.stopRoller()
                )

                // Commands.sequence(
                //     // superSystem.moveTo(PositionEquivalents.L5),
                //     superSystem.moveTo(PositionEquivalents.L1),
                //     superSystem.moveTo(PositionEquivalents.SemiStow)
                // )
                )
            );
    }
}
