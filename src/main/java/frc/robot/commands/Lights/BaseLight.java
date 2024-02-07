package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.subsystems.Lights.Lights;

public class BaseLight extends Command implements Comparable<BaseLight> {
  public final LightsType m_type;
  private final int m_duration;
  public final Lights m_lights;
  private Timer m_timer = new Timer();

  /**
   * constructor description
   * 
   * @param duration in seconds
   */
  public BaseLight(LightsType type, int duration, Lights lights) {
    m_type = type;
    m_duration = duration;
    m_lights = lights;
  }

  public BaseLight(LightsType type, Lights lights) {
    m_type = type;
    m_duration = 0;
    m_lights = lights;
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_lights.queueCommand(this);
  }

  public void once() {

  }

  public void update() {
  }

  @Override
  public boolean isFinished() {
    if (m_duration == 0) {
      return false;
    } else {
      return m_timer.hasElapsed(m_duration);
    }
  }

  @Override
  public int compareTo(BaseLight o) {
    if (m_type.ordinal() > o.m_type.ordinal()) {
      return 1;
    } else if (m_type.ordinal() == o.m_type.ordinal()) {
      return 0;
    } else {
      return -1;
    }
  }

}
