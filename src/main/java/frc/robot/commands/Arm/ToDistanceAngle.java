package frc.robot.commands.Arm;

import frc.robot.Constants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.Swerve;

public class ToDistanceAngle extends ToAngle {
  private Swerve m_drive;
  private ArmEndBehaviour m_endBehaviour;

  public static enum ArmEndBehaviour {
    NORMAL,
    NEVER_ENDING
  }

  public ToDistanceAngle(Swerve drive, Arm arm) {
    super(() -> {
      // var target =
      // Constants.ArmConstants.armAngleInterpolation.getTarget(drive.getDistanceFromAmp());
      var target = Constants.ArmConstants.armAngleInterpolationPolynominalRegression
          .getPrediction(drive.getDistanceFromSpeaker());

      return target;
    }, arm);
    m_drive = drive;
    m_endBehaviour = ArmEndBehaviour.NORMAL;
  }

  public ToDistanceAngle(Swerve drive, Arm arm, ArmEndBehaviour endBehaviour) {
    super(() -> Constants.ArmConstants.armAngleInterpolationPolynominalRegression
        .getPrediction(drive.getDistanceFromSpeaker()), arm);
    m_drive = drive;
    m_endBehaviour = endBehaviour;
  }

  @Override
  public boolean isFinished() {
    switch (m_endBehaviour) {
      case NORMAL:
        return super.isFinished();
      case NEVER_ENDING:
        return false;
      default:
        return super.isFinished();
    }
  }
}
