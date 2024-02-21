package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leader = new CANSparkMax(Constants.ShooterConstants.leader, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.follower, MotorType.kBrushless);

  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(Constants.ShooterConstants.shooterFeedforward[0],
      Constants.ShooterConstants.shooterFeedforward[0]);

  private double velocitySetpoint = 0;
  private double velocityRateOfChange = 0;

  private LoggedTunableNumber shooterKp = new LoggedTunableNumber("shooterKp",
      Constants.ShooterConstants.shooterPID[0]);
  private LoggedTunableNumber shooterKi = new LoggedTunableNumber("shooterKi",
      Constants.ShooterConstants.shooterPID[1]);
  private LoggedTunableNumber shooterKd = new LoggedTunableNumber("shooterKd",
      Constants.ShooterConstants.shooterPID[2]);
  private LoggedTunableNumber shooterKs = new LoggedTunableNumber("shooterKs",
      Constants.ShooterConstants.shooterFeedforward[0]);
  private LoggedTunableNumber shooterKv = new LoggedTunableNumber("shooterKv",
      Constants.ShooterConstants.shooterFeedforward[1]);

  public Shooter() {
    setupMotor();
  }

  public void setupMotor() {
    leader.restoreFactoryDefaults();
    leader.setInverted(true);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(40);

    follower.restoreFactoryDefaults();
    follower.setInverted(true);
    follower.enableVoltageCompensation(12.0);
    follower.setSmartCurrentLimit(40);

    follower.follow(leader, true);

    encoder.setVelocityConversionFactor(1);

    pid.setP(Constants.ShooterConstants.shooterPID[0]);
    pid.setI(Constants.ShooterConstants.shooterPID[1]);
    pid.setD(Constants.ShooterConstants.shooterPID[2]);

    pid.setIMaxAccum(0.02, 0);
  }

  public void resetI() {
    pid.setIAccum(0);
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (shooterKp.hasChanged() || shooterKi.hasChanged() || shooterKd.hasChanged()) {
      pid.setP(shooterKp.get());
      pid.setI(shooterKi.get());
      pid.setD(shooterKd.get());
    }
    if (shooterKs.hasChanged() || shooterKv.hasChanged()) {
      ffModel = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
    }
  }

  public void runVelocity(double rpm, double rpmPerSecond) {
    this.velocitySetpoint = rpm;
    this.velocityRateOfChange = rpmPerSecond;
  }

  public void stop() {
    runVelocity(0, 0);
  }

  public double getActualRPM() {
    return encoder.getVelocity();
  }

  public double getDesiredRPM() {
    return velocitySetpoint;
  }

  public double getDesiredRPMPerSecond() {
    return velocityRateOfChange;
  }

  public boolean atSetpoint() {
    return Math.abs(getActualRPM() - velocitySetpoint) < Constants.ShooterConstants.toleranceRPM;
  }

  public void logValues() {
    SmartDashboard.putNumber("Shooter Actual RPM", getActualRPM());
    SmartDashboard.putNumber("Shooter Desired RPM", getDesiredRPM());
    SmartDashboard.putNumber("Shooter Desired RPM/s", velocityRateOfChange);
  }

  @Override
  public void periodic() {
    double feedforward = ffModel.calculate(velocitySetpoint, velocityRateOfChange);
    pid.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);

    checkTunableValues();
    logValues();
  }
}
