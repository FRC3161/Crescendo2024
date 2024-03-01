package frc.robot.subsystems.Lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  public int[] colors = new int[] { 0, 0, 0 };

  public Lights() {
    led = new AddressableLED(Constants.LightsConstants.port);
    buffer = new AddressableLEDBuffer(Constants.LightsConstants.length);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void clearBuffer() {
    colors = Constants.LightsConstants.Colors.GOLD;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < Constants.LightsConstants.length; i++) {
      setRGB(i, colors[0], colors[1], colors[2]);
    }
    sendBuffer();
    logValues();
  }

  private void logValues() {
  }

  public void sendBuffer() {
    led.setData(buffer);
  }

  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, g, r, b);
  }

  public void setHSV(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }
}
