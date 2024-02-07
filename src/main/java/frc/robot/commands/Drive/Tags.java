package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive.Swerve;

public class Tags {
  public static Command DriveToClosestTag(Transform2d distance, Swerve swerve) {
    var closestTag = swerve.getClosestTag();
    if (closestTag.isEmpty())
      return Commands.none();

    var tagPose = swerve.vision.kTagLayout.getTagPose(closestTag.get().getFiducialId());
    var targetPose = tagPose.get().toPose2d().plus(distance);

    return DriveToLocation.driveTo(targetPose, swerve);
  }
}
