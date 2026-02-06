package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.generated.SwerveConstants;
import frc.robot.subsystems.*;
public class RobotContainer {

  // Driver controller
  private final PS5Controller m_controller = new PS5Controller(0);

  // Gyro supplier created via factory and constants
  private final GyroSupplier m_gyro =
      GyroFactory.createGyro(
          SwerveConstants.GyroConstants.GYRO_TYPE,
          SwerveConstants.GyroConstants.GYRO_PARAMS);

  // Swerve drivetrain subsystem
  private final DrivetrainSubsystem m_swerve = new DrivetrainSubsystem(m_gyro::getRotation2d, new Pose2d());
  private final PathCommands pathCommands = new PathCommands();
  private final SubsystemCommands subsystemCommands = new SubsystemCommands();

  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() { //TODO: CONFIGURE BUTTON BINDINGS!!!, fix susbystem organization
    // Example for later:
     //m_controller.cross(null).while(subsystemCommands.BlankCommand().schedule()); // doesn't do anything, placeholder
     // - till subsystem commands are connected to real motors
  }

  private void configureDefaultCommands() {
    // Default drive command: run every scheduler cycle in teleop
    m_swerve.setDefaultCommand(
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
                      * DrivetrainSubsystem.kMaxAngularSpeed;

              // Command the drivetrain. 0.02 is the nominal TimedRobot loop period (20 ms).
              m_swerve.drive(xSpeed, ySpeed, rot, true, 0.02);
              // m_simSwerve.drive(xSpeed, ySpeed, rot, true, 0.02);
            },
            m_swerve));
  }

  /** Replace this with your real autonomous routine later. */
  public Command getAutonomousCommand() {
    // e.g. return Autos.exampleAuto(m_swerve);
    return null;
  }

  // Optional: expose drivetrain / controller if you need them elsewhere
  public DrivetrainSubsystem getDrivetrain() {
    return m_swerve;
  }

  public PS5Controller getDriverController() {
    return m_controller;
  }
}
