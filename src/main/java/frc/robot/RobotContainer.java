package frc.robot;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.studica.frc.Navx;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Swerve.SwerveConstants;
import frc.robot.Swerve.SwerveModuleSimulation;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem.*;
import frc.robot.subsystems.IndexerSubsystem.*;
import frc.robot.subsystems.IntakeSubsystem.*;
import frc.robot.subsystems.ShooterSubsystem.*;
import frc.robot.subsystems.VisionSubsystem.*;
import frc.robot.util.PathUtil;
public class RobotContainer {
  private final Navx navX = new Navx(0, 100); // rate in Hz
  private final Drivetrain drivetrain = new Drivetrain(() -> navX.getRotation2d().unaryMinus(), new Pose2d());  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());
  //private final Indexer indexer = new Indexer();
  //private final Intake intake = new Intake();
  //private final Shooter shooter = new Shooter(); // make one for hood separate from shooter?
  private final Vision vision = new Vision();
//  
  private final PS5Controller m_controller = new PS5Controller(0);
  //navX.enableOptionalMessages(true, false, false, false, false, false, false, false, false);
  //inputs.yawPosition = navX.getRotation2d().unaryMinus();
  //
  private final Robot robot = new Robot();
  private final PathUtil pathUtil = new PathUtil();
  private final SubsystemCommands subsystemCommands = new SubsystemCommands();

  //private final DrivetrainSubsystem m_swerve = SubsystemCommands.drivetrainSubsystem;//new DrivetrainSubsystem(() -> Rotation2d.fromDegrees(gyro.getYaw()), new Pose2d());  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());
  private final SwerveModuleSimulation swerveModuleSim = new SwerveModuleSimulation();
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // 
  private Rotation2d zeroRotation = Rotation2d.kZero;
  public final PhotonCamera camera0; // needs callibrated
  public final PhotonCamera camera2;
  //public record cameraData = visionSubsystem.cameraData; // FIXXXXX

  //public cameraData curCameraResults;
  Timer timer;
  //Timer timer = new Timer();
  AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static List<Pose3d> AprilTagPoses;

  //
  List<Integer> aprilTagIDs = Arrays.asList(1, 2, 3); // do we need this?????? maybe get rid of it <---------------------
  public static int curAprilTagID = 0;
  public static boolean targetVisible = false;
  public static double targetYaw = 0;
  public static double targetRange; // from photonvision docs
  public static double kPVision_Turn;
  
  Pose2d curPose;
  double curX;
  double curY;
  //Rotation2d curRot;

  int curPathStep = 1;

  public static boolean pathRunning = false;
  // Gyro supplier created via factory and constants
  private final GyroSupplier m_gyro =
      GyroFactory.createGyro(
          SwerveConstants.GyroConstants.GYRO_TYPE,
          SwerveConstants.GyroConstants.GYRO_PARAMS);

  // Swerve drivetrain subsystem
  private final PathCommands pathCommands = new PathCommands();
  //private final SubsystemCommands subsystemCommands = new SubsystemCommands();

  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
    //
    navX.enableOptionalMessages(true, false, false, false, false, false, false, false, false);

    //
    pathRunning = false;
    SmartDashboard.putBoolean("running Path1Command",true);
    AprilTagPoses = Arrays.asList();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    camera0 = new PhotonCamera("PC_Camera0");
    camera2 = new PhotonCamera("PC_Camera2");
    Rotation2d originRot = new Rotation2d(0);
    Pose2d origin = new Pose2d(0,0,originRot);
    drivetrain.resetOdometry(origin);
    //
    
    for (int i = 1; i < 33; i++) { // 33 because 32 tags, index 0 will return a safe Null
        Pose3d tagPose = kTagLayout.getTagPose(i).orElse(new Pose3d()); 
        SmartDashboard.putNumber("tagPose X",tagPose.getX());
        //AprilTagPoses.add(tagPose);
    }
  }

  private void configureBindings() { //TODO: CONFIGURE BUTTON BINDINGS!!!, fix subsystem organization
    // Example for later:
    new JoystickButton(m_controller, PS5Controller.Button.kCircle.value) //getting path from current visible tag(s)
    //- in case of multiple, it'll use the last one in the results sequence
         .whileTrue(pathUtil.getPathFromTagID(
            vision.getCameraResults().AprilTagID(), drivetrain, true, robot.getPeriod(), robot, targetYaw));
  }

  private void configureDefaultCommands() {
    // Default drive command: run every scheduler cycle in teleop
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> {
              // Get the x speed. We are inverting this because Xbox controllers return
              // negative values when we push forward.
              final var xSpeed =
                  -m_xspeedLimiter.calculate(
                          MathUtil.applyDeadband(m_controller.getLeftY(), 0.05))
                      * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

              // Get the y speed or sideways/strafe speed. We are inverting this because
              // we want a positive value when we pull to the left. Xbox controllers
              // return positive values when you pull to the right by default.
              final var ySpeed =
                  -m_yspeedLimiter.calculate(
                          MathUtil.applyDeadband(m_controller.getLeftX(), 0.05))
                      * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

              // Get the rate of angular rotation. We are inverting this because we want a
              // positive value when we pull to the left (remember, CCW is positive in
              // mathematics). Xbox controllers return positive values when you pull to
              // the right by default.
              final var rot =
                  -m_rotLimiter.calculate(
                          MathUtil.applyDeadband(m_controller.getRightX(), 0.05))
                      * Drivetrain.kMaxAngularSpeed;

              // Command the drivetrain. 0.02 is the nominal TimedRobot loop period (20 ms).
              drivetrain.drive(xSpeed, ySpeed, rot, true, 0.02);
              // m_simSwerve.drive(xSpeed, ySpeed, rot, true, 0.02);
            },
            drivetrain));
  }

  /** Replace this with your real autonomous routine later. */
  public Command getAutonomousCommand() {
    // e.g. return Autos.exampleAuto(m_swerve);
    return null;
  }

  // Optional: expose drivetrain / controller if you need them elsewhere
  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public PS5Controller getDriverController() {
    return m_controller;
  }
  public Navx getGyro() {
    return navX;
  }
}
