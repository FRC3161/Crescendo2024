package frc.robot.commands.Shooter;

import frc.robot.Constants.ShooterConstants.FeedMode;
import frc.robot.subsystems.Shooter.Feeder;

public class FeedOut extends FeedIn {
  public FeedOut(Feeder feeder) {
    super(feeder);
  }

  @Override
  public void initialize() {
    m_feeder.setFeedMode(FeedMode.OUT);
  }
}
