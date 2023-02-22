package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class armControl extends CommandBase {

    private Arm m_arm;
    private DoubleSupplier firstStageSup;
    private DoubleSupplier secondStageSup;
    
    public armControl(Arm arm, DoubleSupplier firstStageSup, DoubleSupplier secondStageSup){
        this.m_arm = arm;
        this.firstStageSup = firstStageSup;
        this.secondStageSup = secondStageSup;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.controlArm(firstStageSup.getAsDouble(), secondStageSup.getAsDouble());
        
    }
    
}