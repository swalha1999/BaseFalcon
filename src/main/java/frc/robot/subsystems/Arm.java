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

    private int status;  // 0 = Arm closed, 1 = Ground Intake , 2 = Arm in Up position, 3 = Arm in Down position
    private int desiredStatus;

    // private double x = 0.05;
    // private double y = 0;

    // private double xPrev = 0.05;
    // private double yPrev = 0; 

    private double arm1Length = 0.65;
    private double arm2Length = 0.76;


    public Arm(){
        
        ArmFirstStageFXConfig.slot0.kP = 0.05;
        ArmFirstStageFXConfig.slot0.kI = 0.004;
        ArmFirstStageFXConfig.slot0.kD = 0.05;
        ArmFirstStageFXConfig.slot0.kF = 0.0;
        

        ArmFirstStageFXConfig.slot0.integralZone = 2000;
        ArmFirstStageFXConfig.peakOutputForward = 0.45;
        ArmFirstStageFXConfig.peakOutputReverse = -0.1;
        
        // for the second stage
        kP = 0.01; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.7; 
        kMinOutput = -0.5;

        // firstStage = new TalonFX(20, "canivore");
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

    public  void controlArm(double persent, double persent2){
        
        if( abs(persent) > 0.2 && abs(persent2) > 0.1 ){
            firstStage.set(ControlMode.PercentOutput, persent/2.5);
            
            m_pidController.setReference(persent2, CANSparkMax.ControlType.kDutyCycle);
            secondStage.set(persent2/2.5);

            postion1 = firstStage.getSelectedSensorPosition();
            postion2 = m_encoder.getPosition();
        
        }else if(abs(persent2) > 0.1){
        
            firstStage.set(ControlMode.Position, postion1);
            
            secondStage.set(persent2/2.5);
            postion2 = m_encoder.getPosition();
        }else if (abs(persent) > 0.1){
            
            firstStage.set(ControlMode.PercentOutput, persent/2.5);
            postion1 = firstStage.getSelectedSensorPosition();
            
            m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);
        }else{
        
            firstStage.set(ControlMode.Position, postion1);
            m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);
        }

    }


    // public  void controlArm(double persent, double persent2){
        
    //     if( abs(persent) > 0.2 ){
    //         x += persent/1000;
    //     }

    //     if (abs(persent2) > 0.2){
    //         y+= persent2/1000;
    //     }

    //     if(x<0){
    //         x = 0; 
    //     }

    //     if ( (x*x + y*y) > (arm1Length + arm2Length)* (arm1Length+ arm2Length)){
    //         x = xPrev;
    //         y = yPrev;
    //     }

    //     xPrev = x;
    //     yPrev = y;

    //     postion1 = calculate_arm1_postion(x, y);
    //     postion2 = calculate_arm2_postion(x, y);
    //     firstStage.set(ControlMode.Position, postion1);
    //     m_pidController.setReference(postion2, CANSparkMax.ControlType.kPosition);

    // }

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

    public void set_status(int status){
        this.status = status;
    }

    public void set_desired_status(int desiredStatus){
        this.desiredStatus = desiredStatus;
    }

    public int get_status(){
        return status;
    }

    public int get_desired_status(){
        return desiredStatus;
    }

    public double calculate_angle2(double x, double y){
        return (Math.acos((x*x + y*y - arm1Length*arm1Length - arm2Length*arm2Length)/(2*arm1Length*arm2Length)));        
    }

    public double calculate_angle1(double x, double y){
        return Math.atan2( y , x ) - Math.atan2( (arm2Length*Math.sin(calculate_angle2(x, y))) , (arm1Length + arm2Length*Math.cos(calculate_angle2(x, y))));
    }

    public double calculate_arm1_postion(double x, double y){
        return calculate_angle1(x, y) *  409600 / 360;
    }

    public double calculate_arm2_postion(double x, double y){
        return (calculate_angle2(x, y)) *  250 / 360;
    }
    
    public BooleanSupplier onTargetPostion(){
        return () -> { return firstStage.getSelectedSensorPosition() + 3000 > (postion1)
                    && firstStage.getSelectedSensorPosition() -3000 < (postion1)
                    && m_encoder.getPosition() + 9 > (postion2)
                    && m_encoder.getPosition() -9 < (postion2);
                };
    }

    public BooleanSupplier hasReacetPoint(double point){
        return () -> { return firstStage.getSelectedSensorPosition() + 3000 > (point)
                    && firstStage.getSelectedSensorPosition() -3000 < (point);
                };
    }

    public double get_arm1_angle(){
        //gear ratio is 1:100
        //encoder resolution is 4096
        return firstStage.getSelectedSensorPosition() / 409600 * 360 ;
    }

    public double get_arm2_angle(){
        //gear ratio is 1:250
        //encoder resolution is 48
        return m_encoder.getPosition() / 250 * 360 ;
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
        // SmartDashboard.putNumber("Arm 1 Angle", get_arm1_angle());
        // SmartDashboard.putNumber("Arm 2 Angle", get_arm2_angle());
        // SmartDashboard.putNumber("Arm x", x);
        // SmartDashboard.putNumber("Arm y", y);
        // SmartDashboard.putNumber("Arm 1 postion", postion1);
        // SmartDashboard.putNumber("Arm 2 postion", postion2);
        // SmartDashboard.putNumber("calcutated angle 1", calculate_angle1(x, y));
        // SmartDashboard.putNumber("calcutated angle 2", 180 - calculate_angle2(x, y));
    }

    
}


