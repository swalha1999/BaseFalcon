package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class PenomaticSubsystem  extends SubsystemBase {
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.REVPH);   

    
    @Override
        public void periodic() {
            pcmCompressor.enableDigital();        
        }
    
}
