package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class beamMessage extends Command {
  protected final Intake m_intake;

  public beamMessage(Intake intake) {
    m_intake = intake;
  }

  @Override
  public boolean isFinished() {
    return !m_intake.beamy2.get();
  }

}
