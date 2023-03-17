package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;




public class VisionSubsystem extends SubsystemBase {
    
        // private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(2.5);
        
        // private static final double LIMELIGHT_FORWARD = Units.inchesToMeters(5.22);
        
        // private static final double CAMERA_HEIGHT_PIXELS = 720;
        // private static final double CAMERA_WIDTH_PIXELS = 960;
        
        // private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
        // private static final double VERTICAL_FOV = Math.toRadians(45.7);
        
        // private static final double LIMELIGHT_HEIGHT = 0.15;
        // private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(0);
        final double TARGET_HEIGHT_METERS = 0.82;

        
        
        // private boolean shooterHasTargets = false;
        private double distanceToTarget = Double.NaN;
        private double angleToTarget = Double.NaN;
        // private double theta;
        
        
        private Rotation2d r2d;
        

        DoubleArraySubscriber botpose;
        NetworkTable lime3;
        

        public VisionSubsystem(Swerve drivetrain) {

            lime3 = NetworkTableInstance.getDefault().getTable("limelight-fast");
            botpose = lime3.getDoubleArrayTopic("botpose").subscribe(new double[] {});
            TurnOffVisionProcessor();            

        }
    
        public double getDistanceToTarget() {
            return distanceToTarget;
        }
    
        public double getAngleToTarget() {
            return angleToTarget;
        }
    
        public boolean shooterHasTargets() {
            return lime3.getEntry("tv").getBoolean(false);
        }

        public double getYaw(){
            return lime3.getEntry("tx").getDouble(0);
    
        }

        public void TurnOffVisionProcessor(){
            lime3.getEntry("camMode").setNumber(1);
            lime3.getEntry("ledMode").setNumber(1);
        }

        public void TurnOnVisionProcessor(){
            lime3.getEntry("camMode").setNumber(0);
            lime3.getEntry("ledMode").setNumber(3);
        }
        

        public void changePipeLine(int x){
            lime3.getEntry("pipeline").setNumber(x);
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
            
            // boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) > 0.5;;
            // shooterHasTargets = hasTarget;
            // double pitch = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
            // double yaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
            // if (hasTarget) {
            //     theta = Math.toRadians(pitch) + LIMELIGHT_MOUNTING_ANGLE;
            //     distanceToTarget = (TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT) / Math.tan(theta);
            //     angleToTarget = drivetrain.getYaw().getRadians() + Math.toRadians(yaw);
            // }
    
        }
    
    }
