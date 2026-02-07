// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// error:
/*Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
Error at org.photonvision.PhotonCamera.verifyVersion(PhotonCamera.java:490): Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
PhotonVision coprocessor at path /photonvision/PC_Camera has not reported a message interface UUID - is your coprocessor's camera started? */
package frc.robot; // TEST WHEN U GET HOME BESTIEEEEEEEEEEEEEEEEEEEEEEEEEE

/*import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;*/
//import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem.Drivetrain;
import frc.robot.subsystems.VisionSubsystem.Vision;
import frc.robot.subsystems.VisionSubsystem.Vision.cameraData;
import frc.robot.util.PathUtil;
import frc.robot.util.logging.LogUtil;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

import frc.robot.Swerve.SwerveConstants;
import frc.robot.Swerve.SwerveModuleSimulation;
//import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;   
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.studica.frc.Navx;

import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  // private final XboxController m_controller = new XboxController(0);
  RobotContainer robotContainer = new RobotContainer();
  Drivetrain drivetrain = robotContainer.getDrivetrain();
  PS5Controller m_controller = robotContainer.getDriverController();
  Navx navX =  robotContainer.getGyro();
  Pose2d curPose;
  double curX;
  double curY;
private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9);

  public Robot() {
    //navX.enableOptionalMessages(true, false, false, false, false, false, false, false, false);
    //
  
  }
  @Override
  public void robotPeriodic() {
      // This runs in all robot modes (disabled, auto, teleop, test)
      drivetrain.periodic();
      LogUtil.getInstance().runUpdateMethods();
      CommandScheduler.getInstance().run();
  } 

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    drivetrain.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    curPose = drivetrain.getPose();
    curX = curPose.getX();
    curY = curPose.getY();
    //curRot = curPose.getRotation();

    if (m_controller.getSquareButtonPressed()) {
        navX.resetYaw();
    }

    if (m_controller.getCrossButton()) {
        drivetrain.setX();
    } else {
    driveWithJoystick(true);
    }
    
    //
    // if (m_controller.getCircleButton()) { // trigger pathCommands without cameras attached
    //     Command command = pathUtil.getPathFromTagID(1, drivetrain, true, getPeriod(), this, targetYaw); // is targetYaw right here?
    //     if (!command.isScheduled()) {
    //         System.out.println("command scheduled");
    //         command.schedule();
    //     }
    //     else {
    //     System.out.println("command already scheduled");
    //     }
    //     pathRunning = true;
    //     System.out.println("circle pressed");
    // }
  }
  //
  @Override
  public void simulationPeriodic() {
        
        drivetrain.updateSimModules();
  }
  
  private void setSwerve(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double a =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double b =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double c =
        m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.04))
            * 1.4;
    drivetrain.drive(a, b, c, fieldRelative, getPeriod());
  }

  private void driveWithJoystick(boolean fieldRelative) {
        fieldRelative = true;
        //targetYaw = 0;
            setSwerve(-m_controller.getLeftY(), -m_controller.getLeftX(), -m_controller.getRightX(), fieldRelative);
  }
  private void manualControl() {
   //drivetrain.manualDrive(m_controller.getLeftY(), m_controller.getRightX());
  }
}