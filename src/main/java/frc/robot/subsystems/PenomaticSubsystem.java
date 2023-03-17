package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;

public class PenomaticSubsystem  extends SubsystemBase {
    
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Solenoid greaber = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);   
    Solenoid lifter = new Solenoid(0, PneumaticsModuleType.CTREPCM, 1);
    DigitalInput toplimitSwitch = new DigitalInput(0);
    private final Timer m_timer = new Timer();

    boolean state = true;
    boolean Liftrstate = true;
    
    @Override
    public void periodic() {
        m_timer.start();
        pcmCompressor.enableDigital();
        if (toplimitSwitch.get() == false && m_timer.get() > 0.5){
            close();
        }
        SmartDashboard.putBoolean("Greper", state);        
    }

    public void open(){
        state = true;
        greaber.set(state);   
        m_timer.reset();
    }
    
    public void close(){
        state = false;
        greaber.set(state);   
    }

    public void up(){
        Liftrstate = !Liftrstate;
        lifter.set(Liftrstate);
    }

    

    
}
