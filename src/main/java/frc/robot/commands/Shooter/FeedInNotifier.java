package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.FeedMode;
import frc.robot.subsystems.Shooter.Feeder;

public class FeedInNotifier extends Command {
  protected Feeder m_feeder;

  public FeedInNotifier(Feeder feeder) {
    m_feeder = feeder;
  }

  @Override
  public boolean isFinished() {
    return !m_feeder.beamy.get();
  }
}
