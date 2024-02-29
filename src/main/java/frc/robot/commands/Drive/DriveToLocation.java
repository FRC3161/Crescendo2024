package frc.robot.commands.Drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;

public class DriveToLocation {
  public static Command driveTo(Pose2d target, Swerve drive) {
    var command = AutoBuilder.pathfindToPose(target,
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    command.addRequirements(drive);

    return command;
  }

  public static Command driveToUnsure(Supplier<Pose2d> target, Swerve drive) {
    return driveTo(target.get(), drive);
  }
}
