package frc.robot.commands.autos;

import java.io.IOException;
import java.util.ArrayList;
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

public class GenericAuto extends SequentialCommandGroup {
    private SuperSystem superSystem;
    
    private List<PathPlannerPath> pathGroup;
    private Pose2d startingPose;
    private Command[] commands;

    public GenericAuto(SwerveDrivetrain swerve, SuperSystem superSystem, String autoPath, int... positions) 
    throws IOException, ParseException {
        this.superSystem = superSystem;

        this.pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(autoPath);
        this.startingPose = pathGroup.get(0).getStartingDifferentialPose();

        // Create auto //
        ArrayList<Command> commandsList = new ArrayList<>();
        int pathIndex = 0;
        for (int i = 0; i < positions.length; i++) {
            // Go to Reef
            commandsList.add(AutoBuilder.followPath(pathGroup.get(pathIndex++)));

            // Move to branch position
            // switch (positions[i]) {
            //     case 1: superSystem.moveTo(NamedPositions.L1); break;
            //     case 2: superSystem.moveTo(NamedPositions.L2); break;
            //     case 3: superSystem.moveTo(NamedPositions.L3); break;
            //     case 4: superSystem.moveTo(NamedPositions.L4); break;
            // }
            
            // Outtake
            commandsList.add(Commands.sequence(
                // superSystem.outtake(),
                Commands.waitSeconds(1.5)//,
                // superSystem.stopRoller()
            ));
            
            // If not last piece, bring coral from Station
            if (i < positions.length-1) {
                commandsList.add(Commands.sequence(
                    // Go to Station
                    // superSystem.moveTo(NamedPositions.SemiStow),
                    AutoBuilder.followPath(pathGroup.get(pathIndex++)),

                    // Move to Station and intake
                    // superSystem.moveTo(NamedPositions.Station),
                    // superSystem.intake(),
                    Commands.waitSeconds(2.5),
                    // superSystem.stopRoller(),

                    // Move to Reef
                    // superSystem.moveTo(NamedPositions.SemiStow),
                    AutoBuilder.followPath(pathGroup.get(pathIndex++))
                ));
            }
        }
        commandsList.add(stopAuto());

        commands = new Command[commandsList.size()];
        for (int i = 0; i < commandsList.size(); i++)
            commands[i] = commandsList.get(i);
        

        // Return auto //
        addCommands(
            Commands.runOnce(() -> swerve.resetGyroFromPoseWithAlliance(startingPose)),
            Commands.runOnce(() -> swerve.resetOdometryWithAlliance(startingPose)),
            runAuto()
        );
    }

    public Command runAuto() {
        // Return auto //
        return Commands.sequence(
            commands
        );
    }

    public Command stopAuto() {
        return Commands.sequence(
            superSystem.stopRoller()//,
            // elevator.stow()
        );
    }
}
