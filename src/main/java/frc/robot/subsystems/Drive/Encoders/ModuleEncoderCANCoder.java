package frc.robot.subsystems.Drive.Encoders;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleEncoderCANCoder implements ModuleEncoder {
  private CANcoder encoder;
  private Rotation2d offset = new Rotation2d();

  public ModuleEncoderCANCoder(int id) {
    encoder = new CANcoder(id);
    setup();
  }

  @Override
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
  }

  public void setup() {
    var config = new MagnetSensorConfigs();
    config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoder.getConfigurator().apply(config);

    // remove any extra stuff from can network
    encoder.getAbsolutePosition().setUpdateFrequency(50);
    encoder.optimizeBusUtilization();
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    double angle = Math.toRadians(360.0 * encoder.getAbsolutePosition().getValueAsDouble() - offset.getDegrees());
    if (angle < 0) {
      angle = Math.PI * 2 + angle;
    }
    return Rotation2d.fromRadians(angle);
  }
}
