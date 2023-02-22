package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class PenomaticSubsystem  extends SubsystemBase {
    
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Solenoid greaber = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);   
    boolean state = true;
    
    @Override
        public void periodic() {
            pcmCompressor.enableDigital();        
        }

    public void open(){
        state = true;
        greaber.set(state);   
    }
    
    public void close(){
        state = false;
        greaber.set(state);   
    }
    
}
