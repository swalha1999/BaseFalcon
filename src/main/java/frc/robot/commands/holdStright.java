package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;



public class holdStright extends SequentialCommandGroup {
    public holdStright(Arm arm, PenomaticSubsystem intake){
        

        // addRequirements(arm, intake);
        // hold the postion for both arm

        // states :  // 0 = Arm closed, 1 = Ground Intake , 2 = Arm in Up position, 3 = Arm in Down position, -1 = unknown
        addCommands(new InstantCommand(() -> { arm.set_desired_status(2); }));
        
        // addCommands(new InstantCommand(() -> { arm.position_control(36000, 44);}));
        // addCommands(new WaitUntilCommand(arm.onTargetPostion()));

        addCommands(new InstantCommand(() -> { arm.position_control(8500, 0);}));
        addCommands(new WaitUntilCommand(arm.onTargetPostion()));

        
        addCommands(new InstantCommand(() -> { arm.position_control(65000, 85);}));
        addCommands(new WaitUntilCommand(arm.onTargetPostion()));

        addCommands(new WaitCommand(1));

        addCommands(new InstantCommand(() -> { arm .position_control(65000, 92);}));
        

        addCommands(new InstantCommand(() -> { arm.set_status(2);}));

    }
    
}
