package frc.robot.subsystems.Drive.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleEncoder {
  public Rotation2d getAbsolutePosition();

  public void setOffset(Rotation2d offset);
}
