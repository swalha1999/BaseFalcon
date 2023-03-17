package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;


public class closeArm extends SequentialCommandGroup {
public closeArm(Arm arm, PenomaticSubsystem intake) {

    // addRequirements(arm, intake);

    // addCommands(new InstantCommand(() -> { arm.position_control(45000, 8);}));
    // addCommands(new WaitUntilCommand(arm.hasReachedReference1(45000)));

    // addCommands(new InstantCommand(() -> { arm.set_second_stage(8); }));
    // addCommands(new WaitUntilCommand(arm.hasReachedReference2(8)));
    
    // addCommands(new InstantCommand(() -> { arm.set_first_stage(36000);}));
    // addCommands(new WaitUntilCommand(arm.hasReachedReference1(36000)));

    // addCommands(new InstantCommand(() -> { arm.set_second_stage(44);}));
    // addCommands(new WaitUntilCommand(arm.hasReachedReference2(44)));
    
    // addCommands(new InstantCommand(() -> { arm.set_second_stage(8); }));
    // addCommands(new WaitUntilCommand(arm.hasReachedReference2(8)));
    
    addCommands(new InstantCommand(() -> { arm.position_control(0, -3); }));
    addCommands(new WaitUntilCommand(arm.onTargetPostion()));   
}
    
}
