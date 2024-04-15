package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FeedMode;

public class Feeder extends SubsystemBase {
  private final CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feeder, MotorType.kBrushless);
  private final CANSparkMax feederTail = new CANSparkMax(Constants.ShooterConstants.feederTail, MotorType.kBrushless);
  private double feedPower = 1.0;
  private double tailSetpoint = 0;
  private final SparkPIDController feederTailPID;
  private final LoggedTunableNumber feedTunableNumber = new LoggedTunableNumber("Feed Power", feedPower);
  private final RelativeEncoder tailEncoder;
  private FeedMode feedMode = FeedMode.OFF;
  public boolean shouldCommandStop = false;
  // public DigitalInput beam = new DigitalInput(ShooterConstants.beamDIO);
  public DigitalInput beamy = ShooterConstants.beam;

  // Tunable values
  private LoggedTunableNumber tailP = new LoggedTunableNumber("Tail P", Constants.ShooterConstants.tailPID[0]);
  private LoggedTunableNumber tailI = new LoggedTunableNumber("Tail I", Constants.ShooterConstants.tailPID[1]);
  private LoggedTunableNumber tailD = new LoggedTunableNumber("Tail D", Constants.ShooterConstants.tailPID[2]);
  private LoggedTunableNumber tailFF = new LoggedTunableNumber("Tail FF", Constants.ShooterConstants.tailFF);
  private LoggedTunableNumber tailSPEED = new LoggedTunableNumber("Tail Speed", Constants.ShooterConstants.tailSpeed);

  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0, Constants.ShooterConstants.tailFF);

  public Feeder() {
    setupMotor();

    feederTailPID = feederTail.getPIDController();
    tailEncoder = feederTail.getEncoder();
    tailEncoder.setVelocityConversionFactor(1 / 6);
    feederTailPID.setIMaxAccum(0.02, 0);
    // feederTailPID.setOutputRange(0, 9999);

    tailEncoder.setPosition(0);
  }

  public void setupMotor() {
    feeder.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(feeder, Usage.kMinimal);
    feeder.setInverted(true);
    feeder.enableVoltageCompensation(12.0);
    feeder.setSmartCurrentLimit(30, 30);
    feeder.setInverted(true);
    feeder.setIdleMode(IdleMode.kBrake);

    feederTail.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(feederTail, Usage.kVelocityOnly);
    feederTail.setInverted(true);
    feederTail.enableVoltageCompensation(12.0);
    feederTail.setSmartCurrentLimit(30, 30);
    feederTail.setInverted(true);
    feederTail.setIdleMode(IdleMode.kBrake);
  }

  public void burnToFlash() {
    feeder.burnFlash();
    feederTail.burnFlash();
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;
    if (tailP.hasChanged() || tailI.hasChanged() || tailD.hasChanged()) {
      feederTailPID.setP(tailP.get());
      feederTailPID.setI(tailI.get());
      feederTailPID.setD(tailD.get());
    }

    if (tailFF.hasChanged()) {
      ffModel = new SimpleMotorFeedforward(0, tailFF.get());
    }

  }

  public void setFeedMode(FeedMode mode) {
    feedMode = mode;
  }

  public void resetI() {
    feederTailPID.setIAccum(0);
  }

  public boolean isTailAtSetpoint() {
    return Math.abs(tailEncoder.getVelocity() - tailSetpoint) < Constants.ShooterConstants.tailTolerance;
  }

  @Override
  public void periodic() {
    checkTunableValues();
    SmartDashboard.putBoolean("Beamy", beamy.get());
    switch (feedMode) {
      case HP:
        if (!beamy.get()) {
          feeder.set(0);
          tailSetpoint = 0;
          shouldCommandStop = true;
        } else {
          tailSetpoint = -1710;
          feeder.set(feedPower);
        }
        break;
      case INFEED:
        if (!beamy.get()) {
          feeder.set(0);
          tailSetpoint = 0;
          shouldCommandStop = true;
        } else {
          tailSetpoint = 1710;
          feeder.set(feedPower);
        }
        break;
      case CONTINUE:
        feeder.set(feedPower);
        tailSetpoint = 1710;
      case SHOOTFEED:
        if (beamy.get()) {
          feeder.set(0);
          shouldCommandStop = true;
        } else {
          feeder.set(feedPower);
        }
        break;
      case OUT:
        tailSetpoint = tailSPEED.get();
        if (isTailAtSetpoint()) {
          feeder.set(-feedPower);
        }
        break;
      case OFF:
        feeder.set(0);
        tailSetpoint = 0;
        shouldCommandStop = true;
        break;
      default:
        break;
    }

    double ffOutput = ffModel.calculate(tailSetpoint);
    feederTailPID.setReference(tailSetpoint, ControlType.kVelocity, 0, ffOutput);

    SmartDashboard.putNumber("Tail Actual RPM", tailEncoder.getVelocity());
    SmartDashboard.putNumber("Tail Desired RPM", tailSetpoint);

  }

}
