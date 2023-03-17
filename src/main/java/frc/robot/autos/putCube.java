package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.closeArm;
import frc.robot.commands.holdStright;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;

public class putCube extends SequentialCommandGroup {
    
    public putCube(Arm arm, PenomaticSubsystem penomaticSubsystem){
        addCommands(new holdStright(arm, penomaticSubsystem));
        addCommands(new InstantCommand(() -> { arm.position_control(63000, 80);}));
        addCommands(new WaitCommand(0.5));
        addCommands(new InstantCommand(() -> penomaticSubsystem.open()));
        addCommands(new closeArm(arm, penomaticSubsystem));
    }
}
