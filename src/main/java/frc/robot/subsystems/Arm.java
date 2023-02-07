package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {
    
    public TalonFXConfiguration ArmFirstStageFXConfig = new TalonFXConfiguration();
    private TalonFX firstStage;
    private CANSparkMax secondStage;

    boolean is_postional;
    double postion1;
    double postion2;

    
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;

    public Arm(){
        
        ArmFirstStageFXConfig.slot0.kP = 0.05;
        ArmFirstStageFXConfig.slot0.kI = 0.004;
        ArmFirstStageFXConfig.slot0.kD = 0.05;
        ArmFirstStageFXConfig.slot0.kF = 0.0;
        ArmFirstStageFXConfig.slot0.integralZone = 2000;
        ArmFirstStageFXConfig.peakOutputForward = 0.4;
        ArmFirstStageFXConfig.peakOutputReverse = -0.1;
        
        // for the second stage
        kP = 0.01; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.7; 
        kMinOutput = -0.5;

        firstStage = new TalonFX(20);
        firstStage.configFactoryDefault();
        firstStage.configAllSettings(ArmFirstStageFXConfig);
        
        firstStage.setNeutralMode(NeutralMode.Brake);
        firstStage.setSelectedSensorPosition(0);
        
        secondStage = new CANSparkMax(2, MotorType.kBrushless);
        m_pidController = secondStage.getPIDController();
        m_encoder = secondStage.getEncoder();

        m_encoder.setPosition(0);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        
        is_postional = false;   
        postion1 = 0;
        postion2 = 0;

    }

    public  void controlFirstStage(double persent, double up, double down){
        is_postional = false;
        firstStage.set(ControlMode.PercentOutput,persent);
        secondStage.set(up-down);
    }

    public void position_control(double postion1, double postion2 ){
        is_postional = true;
        this.postion1 = postion1;
        this.postion2 = postion2;
    }

    public double get_position(){
        return this.postion1;
    }

    @Override
    public void periodic(){
        if(is_postional){
            firstStage.set(ControlMode.Position, postion1);
            m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);
        }

        SmartDashboard.putNumber("Arm 1 Encoder", firstStage.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm 2 Encoder", m_encoder.getPosition());
    }

    
}


