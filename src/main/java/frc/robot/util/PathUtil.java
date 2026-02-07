// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

//import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
//import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PathUtil {
  //
  private final PathCommands pathCommands = new PathCommands();
  //public record Data(Command Commands,List<Double> targetPose) {};
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public PathUtil() {}
  
  public Command getPathFromTagID(int aprilTagID, DrivetrainSubsystem drivetrain, Boolean fieldRelative, Double m_period, Robot robot, double targetYaw) {
    // commands for each tag: // use fieldRelative to determine if it should be fieldRelative or just offSet!!!!!!!!!
    if (aprilTagID == 0) {
      return pathCommands.BlankCommand();
    }
    else if (aprilTagID == 1) {
      return pathCommands.Path1Command(drivetrain, fieldRelative, m_period);
    }
    else if (aprilTagID == 2) {
      return pathCommands.Path2Command(drivetrain, fieldRelative, m_period);
    }//DriverStation.getAlliance() add ts to params vvvvv
    else if (aprilTagID == 3) {
      return pathCommands.DriveToCenterFromOrigin(drivetrain, fieldRelative, m_period, robot, m_period);
    }
    /*else if (aprilTagID == 4) {
      return pathCommands.AutoPrototype2(drivetrain, fieldRelative, m_period, robot, targetYaw);
    }
    else if (aprilTagID == 5) {
      return pathCommands.ShootThenClimbAuto(drivetrain, fieldRelative, m_period, robot, targetYaw);
    }*/
    else {
      return pathCommands.BlankCommand();
    }
  }
}