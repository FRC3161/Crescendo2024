package frc.robot.commands.Intake;

import frc.robot.Constants.IntakeConstants.IntakeMode;
import frc.robot.subsystems.Intake.Intake;

public class Outake extends IntakeIn {
  public Outake(Intake intake) {
    super(intake);
  }

  @Override
  public void initialize() {
    m_intake.setOutspeed(-0.7);
    // m_intake.setIntakeMode(IntakeMode.OUT);
  }
}
