package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;



public class holdStright extends SequentialCommandGroup {
    public holdStright(Arm arm, PenomaticSubsystem intake){
        // addRequirements(arm, intake);
        // close the small arm
        addCommands(new InstantCommand(() -> { arm.set_second_stage(24); }));
        addCommands(new WaitUntilCommand(arm.hasReachedReference2(24)));

        // set the large arm 
        addCommands(new InstantCommand(() -> { arm.set_first_stage(37000); }));
        addCommands(new WaitUntilCommand(arm.hasReachedReference1(37000)));

        // hold the postion for both arm
        addCommands(new InstantCommand(() -> { arm.position_control(66953, 125);}));

    }
    
}
