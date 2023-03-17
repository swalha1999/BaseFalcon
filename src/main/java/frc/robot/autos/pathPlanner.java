package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.takeFormGround;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class pathPlanner extends SequentialCommandGroup {
    
    public pathPlanner(Swerve s_Swerve, Arm s_Arm, PenomaticSubsystem penomaticSubsystem){
        
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("HumanPlayerSide", new PathConstraints(2.5, 2));
        
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("ArmDown", new takeFormGround(s_Arm, penomaticSubsystem));
        // eventMap.put("intakeDown", new IntakeDown());
        
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.7, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );
         
        Command fullAuto = autoBuilder.fullAuto(examplePath);


        
        addCommands(
            fullAuto
        );

        // reset the odomtry to the location off the robot and start the swerve drive
        
    }
}