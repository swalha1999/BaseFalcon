package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

// import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
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
        ArmFirstStageFXConfig.peakOutputForward = 0.35;
        ArmFirstStageFXConfig.peakOutputReverse = -0.3;
        
        // for the second stage
        kP = 0.01; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.6; 
        kMinOutput = -0.4;

        firstStage = new TalonFX(20, "canivore");
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

    public  void controlArm(double persent, double persent2){
        
        if( abs(persent) > 0.1 && abs(persent2) > 0.1 ){
            firstStage.set(ControlMode.PercentOutput, persent/2);
            
            m_pidController.setReference(persent2, CANSparkMax.ControlType.kDutyCycle);
            secondStage.set(persent2/2);

            postion1 = firstStage.getSelectedSensorPosition();
            postion2 = m_encoder.getPosition();
        
        }else if(abs(persent2) > 0.1){
        
            firstStage.set(ControlMode.Position, postion1);
            
            secondStage.set(persent2/2);
            postion2 = m_encoder.getPosition();
        }else if (abs(persent) > 0.1){
            
            firstStage.set(ControlMode.PercentOutput, persent/2);
            postion1 = firstStage.getSelectedSensorPosition();
            
            m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);
        }else{
        
            firstStage.set(ControlMode.Position, postion1);
            m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);
        }

    }

    public void position_control(double postion1, double postion2 ){
        this.postion1 = postion1;
        this.postion2 = postion2;
    }

    public void set_second_stage(double postion2){
        this.postion2 = postion2;
    }

    public void set_first_stage(double postion1){
        this.postion1 = postion1;
    }

    public BooleanSupplier hasReachedReference2(double reference) {
        return () -> { return secondStage.getEncoder().getPosition() + 5 > (reference)
          && secondStage.getEncoder().getPosition() -5 < (reference); };  
    }

    public BooleanSupplier hasReachedReference1(double reference) {
        return () -> { return firstStage.getSelectedSensorPosition() + 1000 > (reference)
          && firstStage.getSelectedSensorPosition() -1000 < (reference); };  
    }

    public double abs(double num){
        if(num < 0){
            return -num;
        }else{
            return num;
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm 1 Encoder", firstStage.getSelectedSensorPosition());
        SmartDashboard.putNumber("Arm 2 Encoder", m_encoder.getPosition());
    }

    
}


