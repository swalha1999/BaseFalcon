package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class armControl extends CommandBase {

    private Arm m_arm;
    private DoubleSupplier translationSup;
    private DoubleSupplier up;
    private DoubleSupplier down;

    public armControl(Arm arm, DoubleSupplier translationSup, DoubleSupplier up, DoubleSupplier down){
        this.m_arm = arm;
        this.translationSup = translationSup;
        this.down = down;
        this.up = up;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.controlFirstStage(translationSup.getAsDouble(), up.getAsDouble(), down.getAsDouble());
        
    }
    
}