package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.subsystems.Lights.Lights;

public class SolidColor extends Command {
  private final int[] m_color;
  private final Lights m_lights;

  public SolidColor(Lights lights, int[] color) {
    m_color = color;
    m_lights = lights;
  }

  @Override
  public void initialize() {
    m_lights.colors = m_color;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
