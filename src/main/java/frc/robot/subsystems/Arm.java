package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class Arm extends SubsystemBase{
    
    private TalonFX firstStage;
    private VictorSP victor;
    public Arm(){    
        firstStage = new TalonFX(20);
        firstStage.setNeutralMode(NeutralMode.Brake);
        victor = new VictorSP(0);
    }

    public  void controlFirstStage(double persent, double up, double down){
        firstStage.set(ControlMode.PercentOutput, persent);
        victor.set(up-down);

    }

    
}
