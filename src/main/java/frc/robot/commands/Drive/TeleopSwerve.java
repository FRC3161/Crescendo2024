package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  public final Swerve swerve;
  private final DoubleSupplier translationSup;
  public final DoubleSupplier strafeSup;
  public DoubleSupplier rotationSup;

  public TeleopSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  public double[] getJoystickValues() {
    double translationVal, strafeVal, rotationVal;
    /* Get Values, Deadband */
    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband);
    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband);
    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband);

    double[] output = new double[] { translationVal, strafeVal, rotationVal };
    return output;
  }

  @Override
  public void execute() {
    double translationVal, strafeVal, rotationVal;
    double[] joystickValues = this.getJoystickValues();
    translationVal = joystickValues[0] * 0.5;
    strafeVal = joystickValues[1] * 0.5;
    rotationVal = -joystickValues[2];

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal,
            strafeVal),
        rotationVal);
  }
}
