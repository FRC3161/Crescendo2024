package frc.robot.subsystems.Arm.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmEncoder {
  public Rotation2d getAbsolutePosition();

  public void setOffset(Rotation2d offset);
}
