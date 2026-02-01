package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase
{

  private final SparkMax leftShooterMotor = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(16, MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);
  //
  private final SmartMotorController leftShooterMotorController = new SparkWrapper(leftShooterMotor, DCMotor.getNEO(1), motorConfig);
  private final SmartMotorController rightShooterMotorController = new SparkWrapper(rightShooterMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig leftShooterConfig = new FlyWheelConfig(leftShooterMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-500), RPM.of(500))
      .withSpeedometerSimulation(RPM.of(750));
  private final FlyWheelConfig rightShooterConfig = new FlyWheelConfig(rightShooterMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-500), RPM.of(500))
      .withSpeedometerSimulation(RPM.of(750));


  private final FlyWheel leftShooter = new FlyWheel(leftShooterConfig);
  private final FlyWheel rightShooter = new FlyWheel(rightShooterConfig);

  public ShooterSubsystem() {

  }

  public AngularVelocity getLeftShooterVelocity() {return leftShooter.getSpeed();}
  public AngularVelocity getRightShooterVelocity() {return rightShooter.getSpeed();}

  public Command setLeftShooterVelocity(AngularVelocity speed) {return leftShooter.setSpeed(speed);}
  public Command setRightShooterVelocity(AngularVelocity speed) {return rightShooter.setSpeed(speed);}

  public Command setLeftShooterDutyCycle(double dutyCycle) {return leftShooter.set(dutyCycle);}
  public Command setRightShooterDutyCycle(double dutyCycle) {return rightShooter.set(dutyCycle);}

  public Command setLeftShooterVelocity(Supplier<AngularVelocity> speed) {return leftShooter.setSpeed(speed);}
  public Command setRightShooterVelocity(Supplier<AngularVelocity> speed) {return rightShooter.setSpeed(speed);}

  public Command setLeftShooterDutyCycle(Supplier<Double> dutyCycle) {return leftShooter.set(dutyCycle);}
  public Command setRightShooterDutyCycle(Supplier<Double> dutyCycle) {return rightShooter.set(dutyCycle);}

  public Command leftShooterSysId() {return leftShooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}
  public Command rightShooterSysId() {return rightShooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}

  @Override
  public void periodic() {
      leftShooter.updateTelemetry();
      rightShooter.updateTelemetry();
    }

  @Override
  public void simulationPeriodic() {
      leftShooter.simIterate();
      rightShooter.simIterate();
  }
}
