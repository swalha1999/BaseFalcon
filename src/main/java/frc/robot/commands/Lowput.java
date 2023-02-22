package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;



public class Lowput extends SequentialCommandGroup {
    public Lowput(Arm arm, PenomaticSubsystem intake){
        // addRequirements(arm, intake);
        // close the small arm
        addCommands(new InstantCommand(() -> { arm.set_second_stage(13); }));
        addCommands(new WaitUntilCommand(arm.hasReachedReference2(13)));

        // set the large arm 
        addCommands(new InstantCommand(() -> { arm.set_first_stage(23416); }));
        addCommands(new WaitUntilCommand(arm.hasReachedReference1(23416)));

        // hold the postion for both arm
        addCommands(new InstantCommand(() -> { arm.position_control(23416, 44);}));

    }
    
}
