package frc.robot.vision;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {

    public static double timeMS = Timer.getFPGATimestamp();
    final Field2d field = new Field2d();
    private final List<String> limelightNames = List.of("limelight-fright", "limelight-one");//, "limelight-bright", "limelight-bleft");
    private int loopCounter = 0;
    private final CommandSwerveDrivetrain drivetrain;
    
        public VisionSubsystem(CommandSwerveDrivetrain drivetrain) { //constructor
            this.drivetrain = drivetrain;
            SmartDashboard.putData(field); 
        }
        
        private Vector<N3> calculateStdDev(double distance) {

            double xyUncertainty = 0.2 * Math.pow(distance, 2);
            double thetaUncertainty = 0.4 * Math.pow(distance, 2);
            
            
            if (xyUncertainty < 0.1) xyUncertainty = 0.1;
            if (thetaUncertainty < 0.4) thetaUncertainty = 0.4;
    
            return VecBuilder.fill(xyUncertainty, xyUncertainty, thetaUncertainty); 
        }
    
        private void updateVision(String name) {
            if (!LimelightHelpers.getTV(name)) {return;} //if limelight not exist, then dont even bother running the rest
        
            var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            
            if (mt2 == null || mt2.tagCount == 0 || mt2.avgTagDist > 4.0 || 
                mt1 == null || mt1.tagCount == 0 || mt1.avgTagDist > 4.0)
                {return;}
            
            LimelightHelpers.SetRobotOrientation(name, mt1.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
            Vector<N3> stdDevs = calculateStdDev(mt1.avgTagDist);
        
            drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, stdDevs);
        }
        
        public Pose2d getCurrentPose() {
            return drivetrain.getState().Pose;
        }
    
        public double getDistToTarget(Translation2d targetPosition) {
            return drivetrain.getState().Pose.getTranslation().getDistance(targetPosition);
        }
    
        public void printCurrentPose(){
            System.out.println(drivetrain.getState().Pose);
        }
        
        @Override
        public void periodic() {
            timeMS = Timer.getFPGATimestamp();
            String activeCamera = limelightNames.get(loopCounter % limelightNames.size()); //i do this because it is really taxing on the RoboRio to calculate everything and process two limelights at the same time. by doing this, it's switching back and forth between the two which should give me some less lag
            updateVision(activeCamera);
            //updateVision("limelight-fright");
            //updateVision("limelight-one");
            //printCurrentPose();

            if (loopCounter % 2 == 0) {
                field.setRobotPose(drivetrain.getState().Pose);
            }
        
            loopCounter++;
        }
}
