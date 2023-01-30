package frc.robot.commands;

import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.PenomaticSubsystem;

public class openGreper extends CommandBase {

    private PenomaticSubsystem penomaticSubsystem;
    // private BooleanSupplier state;
    public openGreper(PenomaticSubsystem penomaticSubsystem, BooleanSupplier state){
        this.penomaticSubsystem = penomaticSubsystem;
        // this.state = state;

        addRequirements(penomaticSubsystem);
    }

    @Override
    public void execute() {
        penomaticSubsystem.open_close();
        
    }
    
}
