package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem {
    List<List<PhotonPipelineResult>> curCameraResults;
    public record cameraData<T>(Integer AprilTagID, Class<T> type, T value) {}
    public 
    public Double targetYaw = 0.0;

    public getCameraResults() {
        var results = Arrays.asList(camera0.getAllUnreadResults(),camera2.getAllUnreadResults());
        curCameraResults = results;
        for (int i = 0; i < results.size(); i++) { // looping through results of each camera, with this system camera2 has priority, see if you need to coordinate
            // - it so all cameras combine results or if this system works - THIS IS THE PROBLEM THIS NEVER RETURNS TARGET AND VISIBLE <---------
            if (!results.get(i).isEmpty()) {// Camera processed a new frame since last
                // Get the last one in the list.
                var result = results.get(i).get(results.get(i).size() - 1);
            // SmartDashboard.putNumber("Target tag ID", (result.getTargets().get(result.getTargets().size)-1));
                SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
                if (result.hasTargets()) {
                    // At least one AprilTag was seen by the camera - should be getting thru to here on/off but still yes
                    for (var target : result.getTargets()) {
                        if (aprilTagIDs.contains(target.getFiducialId())) { 
                            // found one of the tags in aprilTagIDs
                            curAprilTagID = target.getFiducialId();
                            targetYaw = target.getYaw();
                            targetVisible = true;
                            SmartDashboard.putNumber("Target tag ID", curAprilTagID);
                            SmartDashboard.putNumber("tag vis on camera #",i);
                            System.out.println(target.getYaw());
                            targetRange =
                                        PhotonUtils.calculateDistanceToTargetMeters( // THESE NEED TO BE TUNED???
                                                0.5   , // Measured with a tape measure, or in CAD.
                                                1.435, // From 2024 game manual for ID 22, CHANGE IF U WANT TS TO WORK
                                                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                                Units.degreesToRadians(target.getPitch()));
                        }
                    }
                }
            }
            else {
            }
        }
    }
}
