package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDriveOdometry vissionOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    PIDController PIDController_x;


    // private Pose2d visionPose2d = new Pose2d();
    // private Pose2d visionLastPose2d = new Pose2d();

    public Swerve() {
        PIDController_x = new PIDController(0.25, 0, 0.07);
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        // indentify all the swerve modules and add them to the array
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants, ""),
            new SwerveModule(1, Constants.Swerve.Mod1.constants, ""),
            new SwerveModule(2, Constants.Swerve.Mod2.constants, ""),
            new SwerveModule(3, Constants.Swerve.Mod3.constants, "")
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        vissionOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean align, double tx) {
        SwerveModuleState[] swerveModuleStates;
        if( align ){
            double pidOutput = PIDController_x.calculate(tx,11.5) > 0.7  ? 0.7 : PIDController_x.calculate(tx,11.5);
            pidOutput = pidOutput < -0.7 ? -0.7 : pidOutput;
            swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(
                0,pidOutput,0));

        }
        else{
            swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        }                   
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getVisionPose(){
        return vissionOdometry.getPoseMeters();
    }

    public void resetVisionPose(Pose2d pose){
        vissionOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }



    @Override
    public void periodic(){
        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }

        swerveOdometry.update(getYaw(), getModulePositions());

        // vissionOdometry.update(getYaw(), getModulePositions());  
        // SmartDashboard.putNumber("Sensor 1", mSwerveMods[0].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Sensor 2", mSwerveMods[1].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Sensor 3", mSwerveMods[2].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Sensor 4", mSwerveMods[3].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Vision X",getVisionPose().getX());
        // SmartDashboard.putNumber("Vision Y",getVisionPose().getY());
        
    }
}


