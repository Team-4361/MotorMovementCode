package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Robot extends TimedRobot {
    private PhotonCamera camera;

    @Override
    public void robotInit() {
        camera = new PhotonCamera("photonvision"); // Change this to your actual camera name
    }

    @Override
    public void robotPeriodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get 3D position from PhotonVision
            double x = target.getBestCameraToTarget().getX();  // Left/Right
            double y = target.getBestCameraToTarget().getY();  // Up/Down
            double z = target.getBestCameraToTarget().getZ();  // Forward
            double distance = target.getBestCameraToTarget().getTranslation().getNorm(); // Actual 3D distance
            double roll = target.getBestCameraToTarget().getRotation().getX();
            double pitch = target.getBestCameraToTarget().getRotation().getY();
            double yaw = target.getBestCameraToTarget().getRotation().getZ();

            // Display values on SmartDashboard
            SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
            SmartDashboard.putNumber("X (m)", x);
            SmartDashboard.putNumber("Y (m)", y);
            SmartDashboard.putNumber("Z (Forward) (m)", z);
            SmartDashboard.putNumber("Distance (m)", distance); // Most accurate distance
            SmartDashboard.putNumber("Roll (deg)", roll);
            SmartDashboard.putNumber("Pitch (deg)", pitch);
            SmartDashboard.putNumber("Yaw (deg)", yaw);
        } else {
            SmartDashboard.putString("AprilTag", "No Target Found");
        }
    }
}
