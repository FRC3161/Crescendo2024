package frc.robot.commands.Shooter;

import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Shooter.Shooter;

public class ToDistanceRPM extends ToRPM {
  private Swerve m_drive;

  public ToDistanceRPM(Swerve drive, Shooter shooter) {
    super(() -> {
      var target = Constants.ShooterConstants.shooterRPMInterpolation.getTarget(drive.getDistanceFromSpeaker());
      if (target.isPresent())
        return target.get();

      return shooter.getDesiredRPMleader();
    }, shooter);
    m_drive = drive;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
