package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;




public class exampleAuto extends SequentialCommandGroup {
    // String trajectoryJSON = "output/test1.wpilib.json";
    // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    Trajectory exampleTrajectory;
    
    public exampleAuto(Swerve s_Swerve){
        
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/3,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-1, 0.25), new Translation2d(-1.5, 0.25)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2, 0, Rotation2d.fromDegrees(180)),
                config);
        
        Trajectory exampleTrajectory2 =
                TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0, 0, new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(-1, 0.25), new Translation2d(-1.5, 0.25)),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(-2, 0, Rotation2d.fromDegrees(180)),
                    config);
                    


        // Trajectory exampleTrajectory =
        //         TrajectoryGenerator.generateTrajectory(
        //             // Start at the origin facing the +X direction
        //             new Pose2d(0, 0, new Rotation2d(0)),
        //             // Pass through these two interior waypoints, making an 's' curve path
        //             List.of(
        //                 new Translation2d(0, -1.5),
        //                 new Translation2d(5, -1.5)),
        //             // End 3 meters straight ahead of where we started, facing forward
        //             new Pose2d(4, 0, Rotation2d.fromDegrees(180)),
        //             config);


        // get the trajectory from the json file
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // }

        // config the Angle pid controller
        var thetaController =
            new ProfiledPIDController(
                1, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // genrate the swerve command to follow the tragectory
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        

        SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    exampleTrajectory2,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
            
        // reset the odomtry to the location off the robot and start the swerve drive
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory2.getInitialPose())),
            swerveControllerCommand2
            
            
        );
    }
}