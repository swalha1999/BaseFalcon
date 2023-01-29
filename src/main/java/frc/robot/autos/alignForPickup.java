
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.commands.syncOdomtry;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class alignForPickup extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_drive;
  private final VisionSubsystem m_vision;
  private Pose2d currentRobotPose;  
  private Trajectory alignTrajectory;
  
  public alignForPickup(Swerve swerve, VisionSubsystem vision) {
    m_drive = swerve;
    m_vision = vision;

  
    currentRobotPose = m_vision.getRobotPosition();
    
    
     TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond/4,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared/3)
                .setKinematics(Constants.Swerve.swerveKinematics);


     alignTrajectory = TrajectoryGenerator.generateTrajectory(
            currentRobotPose,
            List.of(new Translation2d(-7+8, 2.7+4), new Translation2d(-7+8, 2+4)),
            new Pose2d(-7.90 + 8, 2 + 4 , Rotation2d.fromDegrees(180)),
            config
            );
 
    var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                alignTrajectory,
                m_drive::getVisionPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);



    addCommands(
        new syncOdomtry(m_drive, m_vision),
        swerveControllerCommand
        );



  }

  
}
