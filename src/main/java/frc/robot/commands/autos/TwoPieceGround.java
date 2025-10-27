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
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SuperSystemConstants.PositionEquivalents;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;


public class TwoPieceGround extends SequentialCommandGroup {
    public TwoPieceGround(String autoname, SuperSystem superSystem, SwerveDrivetrain swerve) throws IOException, ParseException {
        
        List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoname);
        Pose2d startingPose = pathGroup.get(0).getStartingDifferentialPose();

        addCommands(
            Commands.runOnce(swerve.getImu()::zeroAll),
            Commands.waitSeconds(0.1),
            
            Commands.sequence(
                // Move to L4
            
                AutoBuilder.followPath(pathGroup.get(0)),
                superSystem.moveToAuto(PositionEquivalents.L2),
            


                // superSystem.moveToAuto(PositionEquivalents.L4),

                // Outtake
                Commands.waitSeconds(1),
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller(),
                superSystem.moveToAuto(PositionEquivalents.L2),

                // Move to A3O
                
                AutoBuilder.followPath(pathGroup.get(1)),
                Commands.sequence(
                        superSystem.moveToAuto(PositionEquivalents.SemiStow),
                        superSystem.moveTo(PositionEquivalents.GroundIntake)
                ),
            

                Commands.waitSeconds(2),

                // Move to and intake ground coral
                Commands.parallel(
                    AutoBuilder.followPath(pathGroup.get(2)),
                    superSystem.intakeUntilSensed(3)
                ),

                // Move to Reef
                    superSystem.moveToAuto(PositionEquivalents.SemiStow),
                    AutoBuilder.followPath(pathGroup.get(3)),
                    
                    superSystem.moveToAuto(PositionEquivalents.L2),
            
                // superSystem.moveToAuto(PositionEquivalents.L4),

                // Outtake
                Commands.waitSeconds(2),
                superSystem.outtake(),
                Commands.waitSeconds(1),
                superSystem.stopRoller()
            )
        );
    }
}
