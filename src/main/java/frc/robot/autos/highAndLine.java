package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.closeArm;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;
import frc.robot.subsystems.Swerve;

public class highAndLine extends SequentialCommandGroup {
    public highAndLine(Swerve swerve, Arm arm, PenomaticSubsystem penomaticSubsystem){
        addCommands(new putCube(arm, penomaticSubsystem));
        addCommands(new pathPlanner(swerve, arm, penomaticSubsystem));
        // addCommands(new ParallelCommandGroup(new closeArm(arm, penomaticSubsystem), new pathPlanner(swerve, arm, penomaticSubsystem)));
        addCommands(new WaitCommand(0.5));
        addCommands(new InstantCommand( ()->  penomaticSubsystem.close() ));
        addCommands(new InstantCommand( ()->  swerve.zeroGyro() ));
        addCommands(new ParallelCommandGroup(new closeArm(arm, penomaticSubsystem), new goBack(swerve, arm, penomaticSubsystem)));
        addCommands(new putCube(arm, penomaticSubsystem));

        
    }
}
