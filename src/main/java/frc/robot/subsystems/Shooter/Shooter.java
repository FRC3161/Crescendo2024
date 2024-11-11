package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leader = new CANSparkMax(Constants.ShooterConstants.leader, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.ShooterConstants.follower, MotorType.kBrushless);

  private final RelativeEncoder encoder = leader.getEncoder();
  // private final RelativeEncoder Rencoder = follower.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();
  private final SparkPIDController pid2 = follower.getPIDController();
  // private final SparkPIDController Rpid = follower.getPIDController();
  // Uncomment all of Rpid to split the shooter in the code

  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(Constants.ShooterConstants.shooterFeedforward[0],
      Constants.ShooterConstants.shooterFeedforward[1]);
  // private SimpleMotorFeedforward RffModel = new
  // SimpleMotorFeedforward(Constants.ShooterConstants.shooterFeedforward[0],
  // Constants.ShooterConstants.shooterFeedforward[1]);
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
    CANSparkMaxUtil.setCANSparkMaxBusUsage(leader, Usage.kVelocityOnly);
    leader.setInverted(true);
    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(40, 40);
    leader.setInverted(false);

    follower.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(follower, Usage.kVelocityOnly);
    follower.setInverted(true);
    follower.enableVoltageCompensation(12.0);
    follower.setSmartCurrentLimit(40, 40);
    follower.setInverted(true);

    // follower.follow(leader, true);

    encoder.setVelocityConversionFactor(1);

    pid.setP(Constants.ShooterConstants.shooterPID[0]);
    pid.setI(Constants.ShooterConstants.shooterPID[1]);
    pid.setD(Constants.ShooterConstants.shooterPID[2]);

    pid2.setP(Constants.ShooterConstants.shooterPID[0]);
    pid2.setI(Constants.ShooterConstants.shooterPID[1]);
    pid2.setD(Constants.ShooterConstants.shooterPID[2]);

    
    // Rpid.setP(Constants.ShooterConstants.shooterPID[0]);
    // Rpid.setI(Constants.ShooterConstants.shooterPID[1]);
    // Rpid.setD(Constants.ShooterConstants.shooterPID[2]);

    pid.setIMaxAccum(0.04, 0);
        pid2.setIMaxAccum(0.04, 0);

    // Rpid.setIMaxAccum(0.04, 0);
    pid.setOutputRange(0, 9999);
        pid2.setOutputRange(0, 9999);

    // Rpid.setOutputRange(0, 9999);
  }

  public void burnToFlash() {
    follower.burnFlash();
    leader.burnFlash();
  }

  public void resetI() {
    pid.setIAccum(0);
    pid2.setIAccum(0);

    // Rpid.setIAccum(0);
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (shooterKp.hasChanged() || shooterKi.hasChanged() || shooterKd.hasChanged()) {
      pid.setP(shooterKp.get());
      pid.setI(shooterKi.get());
      pid.setD(shooterKd.get());
      pid2.setP(shooterKp.get());
      pid2.setI(shooterKi.get());
      pid2.setD(shooterKd.get());
      // Rpid.setP(shooterKp.get());
      // Rpid.setI(shooterKi.get());
      // Rpid.setD(shooterKd.get());
    }
    if (shooterKs.hasChanged() || shooterKv.hasChanged()) {
      ffModel = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
      // Rffmodel = new SimpleMotorFeedforward(shooterKs.get(), shooterKv.get());
    }
  }

  public void runVelocity(double rpm, double rpmPerSecond) { // Add rpmR
    this.velocitySetpoint = rpm;
    this.velocityRateOfChange = rpmPerSecond;
    // this.RvelocitySetpoint = rpmR;
  }

  public void stop() {
    runVelocity(0, 0);
  }

  public double getActualRPMleader() {
    return encoder.getVelocity();
  }
  // public double getActualRPMFollower() {
  // return Rencoder.getVelocity();
  // }

  public double getDesiredRPMleader() {
    return velocitySetpoint;
  }
  // public double getDesiredRPMfollower() {
  // return RvelocitySetpoint;
  // }

  public double getDesiredRPMPerSecond() {
    return velocityRateOfChange;
  }

  public boolean atSetpoint() {
    return Math.abs(getActualRPMleader() - velocitySetpoint) < Constants.ShooterConstants.toleranceRPM;
  }
  // public boolean atRsetpoint() {
  // return Math.abs(getActualRPMFollower() - velocitySetpoint <
  // Constants.ShooterConstants.toleranceRPM);
  // }

  public void logValues() {
    SmartDashboard.putNumber("Shooter Actual RPM", getActualRPMleader());
    // SmartDashboard.putNumber("Shooter Actual RPM", getActualRPMFollower());

    SmartDashboard.putNumber("Shooter Desired RPM", getDesiredRPMleader());
    SmartDashboard.putNumber("Shooter Desired RPM/s", velocityRateOfChange);

  }

  @Override
  public void periodic() {
    double feedforward = ffModel.calculate(velocitySetpoint, velocityRateOfChange);
    pid.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);
    pid2.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);

    // Rpid.setReference(velocitySetpoint, ControlType.kVelocity, 0, feedforward);
    // double Rfeedforward = Rffmodel.calculate(RvelocitySetpoint,
    // velocityRateOfChange);

    checkTunableValues();
    logValues();
  }
}
