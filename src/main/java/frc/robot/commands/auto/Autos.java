package frc.robot.commands.auto;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.studica.frc.Navx;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.PathCommands;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.DrivetrainSubsystem.Drivetrain;
//
public final class Autos {
    Pose2d robotStartPose;
    static Optional<Alliance> alliance;
    static Boolean alliancePresent = false;
    //
    private Autos(Drivetrain drivetrain, SubsystemCommands subsystemCommands, PhotonCamera camera0, PhotonCamera camera2) {
    //throw new UnsupportedOperationException("don't use this dummy");
        robotStartPose = subsystemCommands.GetStartPoseFromVisibleAprilTags(camera0,camera2, drivetrain);
        drivetrain.resetOdometry(robotStartPose);
        //
        alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
                alliancePresent = true;
            } else {
            }
    }
  //do we have to pass pathCommands? IDEFKATPBRO
  public Command Auto1(Drivetrain drivetrain, PathCommands pathCommands, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw, String leftOrRight, String trenchOrBump) { // figure out how running this is gonna work,
        // - you'll probably need to get rid of the parameters and have the Autos.java file
        // - deal with it itself
        int startGoalTagID = 0;
        int offsetSign = 0;
        double fromHubOffsetX = 2.5; // ADD THESE IN A CONSTANTS FILE TOO???
        double fromHubOffsetY = 0.0; // ADD THESE IN A CONSTANTS FILE TOO???
        if (alliance.get() == Alliance.Red) { // if on red side, add x,y,rot
            startGoalTagID = 9; // or 10
            offsetSign = 1;
        }
        else if (alliance.get() == Alliance.Blue) { // if on blue side, subtract x,y,rot
            startGoalTagID = 25; // or 26
            offsetSign = -1;
        } // add error catcher for if no alliance?
        // also: add something that determines whether 9/10 or 25/26 is better!!!!! or just have an
        // -  auto for each

        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            RobotContainer.AprilTagPoses.get(startGoalTagID).getX()+fromHubOffsetX*offsetSign, // go to a tag
            RobotContainer.AprilTagPoses.get(startGoalTagID).getY()+fromHubOffsetY*offsetSign, // ADD OFFSETS FROM THIS!!!! maybe set their
            //- -/+ sign when setting startGoalTagID
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        ));
        for (int i = 0; i < AutoPoses.AUTO1_POSE2DS.size(); i++) {
            targetPoses.add(AutoPoses.AUTO1_POSE2DS.get(i));
        }
        //

        // DIFFERENTIATE THE AUTO HERE: ADD POSES FOR WHOLE AUTOS IN HERE vvv

        //========================================
        return Commands.sequence( //drive to hub (with offset)
            Commands.run(() -> {
                    List<Double> values = pathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0)); // change 0
                    pathCommands.setSwerve( m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> pathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.run(() -> {
                // run intake for 5 seconds (/fire 8 fuel)
            }).withTimeout(5)
            // TODO NEXT: drive thru trench/over bump to get to neutral zone, vary this here going thru dif sides
            //(make 2 autos branching from here)
            // - so we dont run into our alliance mates
        );
    }
    //
        // only for aligning from in front of hub start, add differentiation! <---------
    /*public Command ShootThenClimbAuto(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        List<Pose2d> targetPoses = Arrays.asList(
            new Pose2d(1.678, 3.75, Rotation2d.fromDegrees(0)),
            new Pose2d(1.438, 3.745, Rotation2d.fromDegrees(-180))
        );
        return Commands.sequence(
            Commands.run(() -> { // align to tag
                PathCommands.AlignToTag(drivetrain, m_period, fieldRelative);
            }).until(() -> (RobotContainer.targetRange <= 2 && RobotContainer.targetYaw <= 5)),
            //
            Commands.run(() -> { // align to front of hub from center start point <- DIFFERENTIATE!!!
                    List<Double> values = PathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    PathCommands.setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> PathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.runOnce(() -> { // shoots all fuel (runs both shooter for 5 sec, maybe add sensor input idk)
                //SubsystemCommands.RunLeftShooter();
                //SubsystemCommands.RunRightShooter();
            }).withTimeout(5),
            //
            Commands.run(() -> { // drives to tower
                    List<Double> values = PathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(1));
                    PathCommands.setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> PathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0)))
            // next put climbing code
        );
    }*/
}
