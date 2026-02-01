package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase
{

  private final SparkMax intakeMotor = new SparkMax(13, MotorType.kBrushless);
  private final SparkMax intakeDeployMotor = new SparkMax(14, MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);
   private final SmartMotorControllerConfig intakeDeployMotorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("IntakeDeployMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  //========
    private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));


  //
  private final SmartMotorController intakeMotorController = new SparkWrapper(intakeMotor, DCMotor.getNEO(1), intakeMotorConfig);
  private final SmartMotorController intakeDeployMotorController = new SparkWrapper(intakeDeployMotor, DCMotor.getNEO(1), intakeDeployMotorConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(intakeMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-500), RPM.of(500))
      .withSpeedometerSimulation(RPM.of(750));
  private final ArmConfig intakeDeployConfig = new ArmConfig(intakeDeployMotorController)
      .withLength(Meters.of(0.135))
      .withHardLimit(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withStartingPosition(Degrees.of(0))
      //.withHorizontalZero(Degrees.of(0))
      .withMechanismPositionConfig(robotToMechanism);


  private final FlyWheel intake = new FlyWheel(intakeConfig);
  private final Arm intakeDeploy = new Arm(intakeDeployConfig);

  public IntakeSubsystem() {

  }

  public AngularVelocity getIntakeVelocity() {return intake.getSpeed();}
  public Angle getIntakeDeployAngle() {return intakeDeploy.getAngle();}

  public Command setIntakeVelocity(AngularVelocity speed) {return intake.setSpeed(speed);}
  public Command setIntakeDeployAngle(Angle angle) {return intakeDeploy.setAngle(angle);}

  public Command setIntakeDutyCycle(double dutyCycle) {return intake.set(dutyCycle);}
  public Command setIntakeDeployDutyCycle(double dutyCycle) {return intakeDeploy.set(dutyCycle);}

  //public Command setLeftShooterVelocity(Supplier<AngularVelocity> speed) {return leftShooter.setSpeed(speed);}
  //public Command setRightShooterVelocity(Supplier<AngularVelocity> speed) {return rightShooter.setSpeed(speed);}

  //public Command setLeftShooterDutyCycle(Supplier<Double> dutyCycle) {return leftShooter.set(dutyCycle);}
  //public Command setRightShooterDutyCycle(Supplier<Double> dutyCycle) {return rightShooter.set(dutyCycle);}

  public Command intakeSysId() {return intake.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}
  public Command intakeDeploySysId() {return intakeDeploy.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}

  @Override
  public void periodic() {
      intake.updateTelemetry();
      intakeDeploy.updateTelemetry();
    }

  @Override
  public void simulationPeriodic() {
      intake.simIterate();
      intakeDeploy.simIterate();
  }
}
