package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;



public class middle extends SequentialCommandGroup {
    public middle(Arm arm, PenomaticSubsystem intake){
        // addRequirements(arm, intake);
        // close the small arm
        addCommands(new InstantCommand(() -> { arm.set_second_stage(88); }));
        addCommands(new WaitUntilCommand(arm.onTargetPostion()));

        // set the large arm 
        addCommands(new InstantCommand(() -> { arm.set_first_stage(56866); }));
        addCommands(new WaitUntilCommand(arm.onTargetPostion()));


    }
    
}
