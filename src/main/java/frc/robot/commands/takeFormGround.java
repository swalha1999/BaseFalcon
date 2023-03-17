package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;


public class takeFormGround extends SequentialCommandGroup {
    public takeFormGround(Arm arm, PenomaticSubsystem intake) {
        // addRequirements(arm, intake);
        
        // Simple cone handoff...
        addCommands(new closeArm(arm, intake)); // make sure the arm is closed
        
        addCommands(new InstantCommand(() -> {intake.open();})); 
        
        addCommands(new InstantCommand(() -> { arm.position_control(-5, 53); }));      
       
    }   
}
