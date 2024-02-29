package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.FeedMode;
import frc.robot.subsystems.Shooter.Feeder;

public class FeedIn extends Command {
  protected Feeder m_feeder;

  public FeedIn(Feeder feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {
    m_feeder.setFeedMode(FeedMode.INFEED);
    m_feeder.shouldCommandStop = false;
  }

  @Override
  public void end(boolean interrupted) {
    m_feeder.setFeedMode(FeedMode.OFF);
  }

  @Override
  public boolean isFinished() {
    return m_feeder.shouldCommandStop;
  }
}
