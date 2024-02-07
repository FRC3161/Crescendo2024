package frc.robot.subsystems.Drive.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;

public class ModuleEncoderThrifty implements ModuleEncoder {
  private AnalogInput encoder;
  private Rotation2d offset = new Rotation2d();

  public ModuleEncoderThrifty(int id) {
    encoder = new AnalogInput(id);
  }

  @Override
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
  }

  private double getRawAbsolutePosition() {
    return encoder.getAverageVoltage() / RobotController.getVoltage5V();
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    double angle = Math.toRadians(360.0 * getRawAbsolutePosition() - offset.getDegrees());
    if (angle < 0) {
      angle = Math.PI * 2 + angle;
    }
    return Rotation2d.fromRadians(angle);
  }
}