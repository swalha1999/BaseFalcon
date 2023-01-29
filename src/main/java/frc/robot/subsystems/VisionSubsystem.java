package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;




public class VisionSubsystem extends SubsystemBase {
    
        // private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);
        
        // private static final double LIMELIGHT_FORWARD = Units.inchesToMeters(5.22);
        
        // private static final double CAMERA_HEIGHT_PIXELS = 720;
        // private static final double CAMERA_WIDTH_PIXELS = 960;
        
        // private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
        // private static final double VERTICAL_FOV = Math.toRadians(45.7);
        
        private static final double LIMELIGHT_HEIGHT = 0.15;
        private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(0);
        final double TARGET_HEIGHT_METERS = 0.82;
        private final Swerve drivetrain;
        
        
        private boolean shooterHasTargets = false;
        private double distanceToTarget = Double.NaN;
        private double angleToTarget = Double.NaN;
        private double theta;
        
        
        private Rotation2d r2d;
        

        DoubleArraySubscriber botpose;
        NetworkTable table;
        

        public VisionSubsystem(Swerve drivetrain) {
            this.drivetrain = drivetrain;
            table = NetworkTableInstance.getDefault().getTable("limelight");
            botpose = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
            ShuffleboardTab tab = Shuffleboard.getTab("Vision");
            
            tab.addBoolean("shooter has targets", () -> shooterHasTargets).withPosition(0, 0).withSize(1, 1);
            tab.addNumber("distance to target", () -> distanceToTarget).withPosition(1, 0).withSize(1, 1);
            tab.addNumber("angle to target", () -> Units.radiansToDegrees(angleToTarget)).withPosition(2, 0).withSize(1, 1);
            tab.addNumber("theta", () -> Math.toDegrees(theta));
            
            tab.addNumber("robot x", () -> getRobotPosition().getX());
            tab.addNumber("robot y", () -> getRobotPosition().getY());
            

        }
    
        public double getDistanceToTarget() {
            return distanceToTarget;
        }
    
        public double getAngleToTarget() {
            return angleToTarget;
        }
    
        public boolean shooterHasTargets() {
            return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
        }

        public void TurnOffVisionProcessor(){
            table.getEntry("camMode").setNumber(1);
        }

        public void TurnOnVisionProcessor(){
            table.getEntry("camMode").setNumber(0);
        }
        
        public void changePipeLine(int x){
            table.getEntry("pipeline").setNumber(x);
        }

        public Pose2d getRobotPosition() {
            try {
                double[] result = botpose.get();
                r2d = Rotation2d.fromDegrees(result[5]);
                 
            
                return new Pose2d(result[0] + 8, result[1] + 4, r2d);

            } catch (Exception e) {
                return new Pose2d();
            }
            
        }

        
        @Override
        public void periodic() {
            
            boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;;
            shooterHasTargets = hasTarget;
            double pitch = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            double yaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
            if (hasTarget) {
                theta = Math.toRadians(pitch) + LIMELIGHT_MOUNTING_ANGLE;
                distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) / Math.tan(theta);
                angleToTarget = drivetrain.getYaw().getRadians() + Math.toRadians(yaw);
            }
    
        }
    
    }
