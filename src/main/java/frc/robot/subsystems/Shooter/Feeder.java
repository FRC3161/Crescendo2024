package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FeedMode;

public class Feeder extends SubsystemBase {
  private final CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feeder, MotorType.kBrushless);
  private double feedPower = 1.0;
  private final LoggedTunableNumber feedTunableNumber = new LoggedTunableNumber("Feed Power", feedPower);
  private FeedMode feedMode = FeedMode.OFF;
  public boolean shouldCommandStop = false;
  // public DigitalInput beam = new DigitalInput(ShooterConstants.beamDIO);
  private DigitalInput beamy = ShooterConstants.beam;

  public Feeder() {
    setupMotor();
  }

  public void setupMotor() {
    feeder.restoreFactoryDefaults();
    feeder.setInverted(true);
    feeder.enableVoltageCompensation(12.0);
    feeder.setSmartCurrentLimit(40);
  }

  public void setFeedMode(FeedMode mode) {
    feedMode = mode;
  }

  public void checkTunableValues() {
    if (feedTunableNumber.hasChanged()) {
      feedPower = feedTunableNumber.get();
    }
  }

  @Override
  public void periodic() {
    checkTunableValues();
    SmartDashboard.putBoolean("Beamy", beamy.get());
    switch (feedMode) {
      case INFEED:
        if (!beamy.get()) {
          feeder.set(0);
          shouldCommandStop = true;
        } else {
          feeder.set(feedPower);
        }
        break;
      case SHOOTFEED:
        if (beamy.get()) {
          feeder.set(0);
          shouldCommandStop = true;
        } else {
          feeder.set(feedPower);
        }
        break;
      case OUT:
        feeder.set(-feedPower);
        break;
      case OFF:
        feeder.set(0);
        shouldCommandStop = true;
        break;
      default:
        break;
    }
  }

}
