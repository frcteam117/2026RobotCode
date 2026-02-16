package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;

public class AutoPoses {
    // ALL THESE POSES ARE AFTER LINE UP AND 8 FUEL SHOOT
    public static final List<Pose2d> AUTO1_POSE2DS = Arrays.asList();
    //===
    public static final Optional<Alliance> alliance = DriverStation.getAlliance();
    public static final String allianceColor = alliance.get().toString(); // check what this returns to see if its "Red"/"Blue"
    public AutoPoses(String leftOrRight, String trenchOrBump) { // this assumes all autos will behave the same
        //- with the same differentiation (IDK IF THATS TRUE)
        if (allianceColor == "Red") { // add all other auto poses in here as well?
            if (leftOrRight == "left") { // go to more decimal points?
                if (trenchOrBump == "trench") {// red AZ, using left trench
                    AUTO1_POSE2DS.add(new Pose2d(13.305,0.666, new Rotation2d()));
                }
                else if (trenchOrBump == "bump") {// red AZ, using left bump
                    AUTO1_POSE2DS.add(new Pose2d(13.305,2.5, new Rotation2d())); // TODO: CONT. ADDING POSES
                }
            }
            else if (leftOrRight == "right") {
                if (trenchOrBump == "trench") {// red AZ, using right trench
                    AUTO1_POSE2DS.add(new Pose2d(13.305,7.389, new Rotation2d()));
                }
                else if (trenchOrBump == "bump") {// red AZ, using right bump
                    AUTO1_POSE2DS.add(new Pose2d(13.305,5.525, new Rotation2d()));
                }
            }
        }
        else if (allianceColor == "Blue") {
            if (leftOrRight == "left") { //
                if (trenchOrBump == "trench") {

                }
                else if (trenchOrBump == "bump") {

                }
            }
            else if (leftOrRight == "right") {
                if (trenchOrBump == "trench") {

                }
                else if (trenchOrBump == "bump") {

                }
            }
        }
        
    }
}
