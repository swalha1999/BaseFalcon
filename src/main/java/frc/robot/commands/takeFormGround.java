package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;

public class takeFormGround extends SequentialCommandGroup {
    public takeFormGround(Arm arm, PenomaticSubsystem intake) {
        // addRequirements(arm, intake);
        
        // Simple cone handoff...
        addCommands(new InstantCommand(() -> { arm.set_second_stage(8); }));
        addCommands(new WaitUntilCommand(arm.hasReachedReference2(8)));
        
        addCommands(new InstantCommand(() -> {intake.open();}));
        
        addCommands(new InstantCommand(() -> {arm.set_first_stage(0);}));
        addCommands(new WaitUntilCommand(arm.hasReachedReference1(0)));
        
        addCommands(new InstantCommand(() -> { arm.position_control(0, 71); }));      

        // addCommands(new InstantCommand(() -> {intake.grabCone(-0.3);}));
        // addCommands(new WaitCommand(0.5));
        // addCommands(n ew InstantCommand(() -> {intake.grabCone(0);}));
    }   
}
