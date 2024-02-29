package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeIn extends Command {
  protected final Intake m_intake;

  public IntakeIn(Intake intake) {
    m_intake = intake;
  }

  @Override
  public void initialize() {
    m_intake.setSpeed(0.7);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

}
