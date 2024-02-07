package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

public class ToDistanceAngle extends ToAngle {
  private Swerve m_drive;

  public ToDistanceAngle(Swerve drive, Arm arm) {
    super(() -> {
      var target = Constants.ArmConstants.armAngleInterpolation.getTarget(drive.getDistanceFromAmp());
      if (target.isPresent())
        return target.get();

      return arm.getSetpoint().getRadians();
    }, arm);
    m_drive = drive;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
