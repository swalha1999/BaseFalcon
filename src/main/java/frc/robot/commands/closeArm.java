package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;


public class closeArm extends SequentialCommandGroup {
public closeArm(Arm arm, PenomaticSubsystem intake) {
    // addRequirements(arm, intake);
    addCommands(new InstantCommand(() -> { arm.set_second_stage(0); }));
    addCommands(new WaitUntilCommand(arm.hasReachedReference2(0)));
    addCommands(new InstantCommand(() -> { arm.position_control(0, 0); }));      
}
    
}
