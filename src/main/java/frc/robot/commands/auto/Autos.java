package frc.robot.commands.auto;
import java.util.Arrays;
import java.util.List;

import com.studica.frc.Navx;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.PathCommands;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.*;
public final class Autos {
    Pose2d robotStartPose;
    private final Navx navX = new Navx(0, 100); 
    SubsystemCommands subsystemCommands = new SubsystemCommands();
    DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(() -> navX.getRotation2d().unaryMinus(), new Pose2d());
    // will instantiating this a bunch of times in different files make it so resetting
    // - the odometry is useless???? IDK DO RESEARCH!!!!
    private Autos() {
    //throw new UnsupportedOperationException("don't use this dummy");
        robotStartPose = subsystemCommands.GetStartPoseFromVisibleAprilTags();
        drivetrainSubsystem.resetOdometry(robotStartPose);
        //
    }
  //
  public Command AutoPrototype1(DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) { // figure out how running this is gonna work,
        // - you'll probably need to get rid of the parameters and have the Autos.java file
        // - deal with it itself
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            RobotContainer.AprilTagPoses.get(3).getX(), // go to a tag
            RobotContainer.AprilTagPoses.get(3).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        ));
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = PathCommands.CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    PathCommands.setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> PathCommands.CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.run(() -> {
                // run intake for 5 seconds
            }).withTimeout(5)
        );
    }
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
