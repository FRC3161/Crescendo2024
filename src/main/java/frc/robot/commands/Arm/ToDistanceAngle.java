package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

public class ToDistanceAngle extends ToAngle {
  private Swerve m_drive;
  private boolean waitForPosition = false;

  public ToDistanceAngle(Swerve drive, Arm arm) {
    super(() -> {
      // var target =
      // Constants.ArmConstants.armAngleInterpolation.getTarget(drive.getDistanceFromAmp());
      var target = Constants.ArmConstants.armAngleInterpolationPolynominalRegression
          .getPrediction(drive.getDistanceFromSpeaker());

      return target;
    }, arm);
    m_drive = drive;
  }

  public ToDistanceAngle(Swerve drive, Arm arm, boolean waitForPosition) {
    super(() -> Constants.ArmConstants.armAngleInterpolationPolynominalRegression
        .getPrediction(drive.getDistanceFromSpeaker()), arm);
    m_drive = drive;
    this.waitForPosition = waitForPosition;
  }

  @Override
  public boolean isFinished() {
    if (waitForPosition) {
      return super.isFinished();
    } else {
      return false;
    }
  }
}
