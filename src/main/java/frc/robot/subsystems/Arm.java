package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


public class Arm extends SubsystemBase {
    
    public TalonFXConfiguration ArmFirstStageFXConfig = new TalonFXConfiguration();
    private TalonFX firstStage;
    private VictorSP victor;

    boolean is_postional;
    double postion;

    


    public Arm(){
        
        ArmFirstStageFXConfig.slot0.kP = 0.02;
        ArmFirstStageFXConfig.slot0.kI = 0.004;
        ArmFirstStageFXConfig.slot0.kD = 0.01;
        ArmFirstStageFXConfig.slot0.kF = 0.0;
        ArmFirstStageFXConfig.slot0.integralZone = 2000;

        firstStage = new TalonFX(20);
        firstStage.configFactoryDefault();
        firstStage.configAllSettings(ArmFirstStageFXConfig);
        
        firstStage.setNeutralMode(NeutralMode.Brake);
        firstStage.setSelectedSensorPosition(0);
        victor = new VictorSP(0);

        is_postional = false;
        postion = 0;
    }

    public  void controlFirstStage(double persent, double up, double down){
        is_postional = false;
        firstStage.set(ControlMode.PercentOutput,persent);
        victor.set(up-down);
    }

    public void position_control(double postion){
        is_postional = true;
        this.postion = postion;
    }

    public double get_position(){
        return this.postion;
    }

    @Override
    public void periodic(){
        if(is_postional){
            firstStage.set(ControlMode.Position, postion);
        }
        SmartDashboard.putNumber("Arm Encoder", firstStage.getSelectedSensorPosition());
    }

    
}


