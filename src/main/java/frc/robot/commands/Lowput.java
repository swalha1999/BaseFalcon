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
       
        // hold the postion for both arm
        addCommands(new InstantCommand(() -> { arm.position_control(20416, 32);}));
        addCommands(new WaitUntilCommand(arm.onTargetPostion()));


    }
    
}
