package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;

public class PenomaticSubsystem  extends SubsystemBase {
    Compressor pcmCompressor = new Compressor(2, PneumaticsModuleType.REVPH);
    Solenoid greaber = new Solenoid(2, PneumaticsModuleType.REVPH, 8);   
    boolean state = true;
    
    @Override
        public void periodic() {
            pcmCompressor.enableDigital();        
        }

    public void open_close(){
        state = !state;
        greaber.set(state);   
    }
    
}
