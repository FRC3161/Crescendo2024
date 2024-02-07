package frc.robot.subsystems.Lights;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LightsConstants.LightsType;
import frc.robot.commands.Lights.BaseLight;
import frc.robot.commands.Lights.SolidColor;

public class Lights extends SubsystemBase {
  private PriorityQueue<BaseLight> queue = new PriorityQueue<BaseLight>();
  private BaseLight currentCommand = null;
  private final SolidColor idleCommand;

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public Lights() {
    led = new AddressableLED(Constants.LightsConstants.port);
    buffer = new AddressableLEDBuffer(Constants.LightsConstants.length);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();

    idleCommand = new SolidColor(LightsType.IDLE, 0, this, new int[] { 179, 134, 27 });
    currentCommand = idleCommand;
    idleCommand.schedule();
  }

  private void setCommand(BaseLight command) {
    currentCommand = command;
    currentCommand.once();
    currentCommand.update();
  }

  @Override
  public void periodic() {
    if (currentCommand == null) {
      if (queue.isEmpty()) {
        setCommand(idleCommand);
      } else {
        setCommand(queue.poll());
      }
    } else {
      currentCommand.update();
      if (currentCommand.isFinished()) {
        currentCommand = null;
        if (queue.isEmpty()) {
          idleCommand.schedule();
        }
      } else {
        if (!queue.isEmpty()) {
          if (queue.peek().compareTo(currentCommand) == 1) {
            setCommand(queue.poll());
          }
        }
      }
    }
    sendBuffer();
    logValues();
  }

  public void queueCommand(BaseLight command) {
    queue.add(command);
  }

  public void clearQueue() {
    queue.clear();
    currentCommand = null;
  }

  private String getCurrentMode() {
    return currentCommand.m_type.toString();
  }

  private void logValues() {
    SmartDashboard.putString("Lights Current Mode", getCurrentMode());
  }

  public void sendBuffer() {
    led.setData(buffer);
  }

  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, r, g, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }
}
