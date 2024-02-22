package frc.robot.commands.Intake;

import frc.robot.subsystems.Intake.Intake;

public class Outake extends IntakeIn {
  public Outake(Intake intake) {
    super(intake);
  }

  @Override
  public void initialize() {
    m_intake.setSpeed(1);
  }
}
