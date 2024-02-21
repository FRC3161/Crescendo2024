package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FeedMode;

public class Feeder extends SubsystemBase {
  private final CANSparkMax feeder = new CANSparkMax(Constants.ShooterConstants.feeder, MotorType.kBrushless);
  private final double feedPower = 0.7;
  private FeedMode feedMode = FeedMode.OFF;

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

  @Override
  public void periodic() {
    switch (feedMode) {
      case IN:
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
