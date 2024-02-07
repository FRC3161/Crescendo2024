package frc.robot.commands.Lights;

import frc.robot.Constants;
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.subsystems.Lights.Lights;

public class SolidColor extends BaseLight {
  private final int[] m_color;

  public SolidColor(LightsType type, int duration, Lights lights, int[] color) {
    super(type, duration, lights);
    m_color = color;
  }

  @Override
  public void once() {
    for (int i = 0; i < Constants.LightsConstants.length; i++) {
      m_lights.setRGB(i, m_color[0], m_color[1], m_color[2]);
    }
  }

}
