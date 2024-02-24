package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FeedMode;

public class Feeder extends SubsystemBase {
  private final CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feeder, MotorType.kBrushless);
  private double feedPower = 0.5;
  private final LoggedTunableNumber feedTunableNumber = new LoggedTunableNumber("Feed Power", feedPower);
  private FeedMode feedMode = FeedMode.OFF;
  private DigitalInput beam;
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
    switch (feedMode) {
      case FIRSTIN:
      // if (!beam.get()) 
        feeder.set(feedPower);
      // }
      // else{
        // feeder.set(0);
      // }
        break;
      case SHOOT:
        feeder.set(feedPower);
        break;
      case OUT:
        feeder.set(-feedPower);
        break;
      case OFF:
        feeder.set(0);
      default:
        break;
    }
  }

}
