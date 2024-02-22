package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Interpolation1D;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static boolean enableTunableValues = true;

  public static class LightsConstants {
    public static int port = 1; // TODO: irl values TBD
    public static int length = 0; // TODO: irl values TBD

    public static enum LightsType {
      ENDGAME,
      CLIMB,
      SHOOTING,
      INTAKE,
      IDLE,
      DISABLED
    }
  }

  public static final class VisionConstants {
    public static String cameraName = "orangepi";
    // The layout of the AprilTags on the field
    public static final Transform3d kRobotToCam = new Transform3d(
        new Translation3d(-Units.inchesToMeters(11.55), 0.0, Units.inchesToMeters(8.17)),
        new Rotation3d(0, Units.degreesToRadians(-30), Math.PI));
    public static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    public static final Vector<N3> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
    public static final Vector<N3> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class Operators {
    public static final int driver = 0;
    public static final int operator = 1;
  }

  public static final class SwerveConstants {

    public static enum DriveMode {
      DriverInput,
      Snap,
    }

    public static final double[] snapPID = { 2, 0, 0 };

    /* Drive Controls */
    public static final double stickDeadband = 0.1;

    /* Gyro */
    public static final int pigeonID = 21;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0); // TODO: measure
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = 150.0 / 7.0; // 150/7:1

    /* Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.2; // meters per second
    public static final double maxAngularVelocity = Math.PI; // rads per second

    /* Neutral Modes */
    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" }; // module #0,
    // #1, #2, #3

    public static final double[] driveSVA = new double[] { 0.3, 2.5, 0.0 };
    public static final double[] drivePID = new double[] { 0.15, 0.0000, 0.00010 };
    public static final double[] anglePID = new double[] { 0.02, 0.0, 0.0005 };

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 0;
      public static final double angleOffset = 31.7;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 14;
      public static final int canCoderID = 1;
      public static final double angleOffset = 165;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 15;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 3;
      public static final double angleOffset = 349.5;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 17;
      public static final int angleMotorID = 18;
      public static final int canCoderID = 2;
      public static final double angleOffset = 31.4;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 4;

    public static final PIDConstants translationPID = new PIDConstants(2, 0, 0);
    public static final PIDConstants rotationPID = new PIDConstants(1.5, 0, 0);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ShooterConstants {

    public static final int leader = 31;
    public static final int follower = 32;
    public static final int feeder = 33;
    public static final double[] shooterPID = { 0.0, 0.0000005, 0 };
    public static final double[] shooterFeedforward = { 0.25, 0.002 };
    public static final int toleranceRPM = 10;
    public static final double maxVelocityPerSecond = 2000; // RPM/s
    public static final double maxAcceleration = 700; // RPM/s^2

    public static enum FeedMode {
      IN,
      OUT,
      OFF
    }

    // Interpolation
    // {metres, rpm}
    public static final Interpolation1D shooterRPMInterpolation = new Interpolation1D(
        new double[] { 1f, 400f },
        new double[] { 5f, 500f },
        new double[] { 8f, 600f },
        new double[] { 10f, 1200f });
  }

  public static final class ArmConstants {
    public static final int leaderID = 21;
    public static final int followerID = 22;
    public static final int encoderID = 0;

    public static final Rotation2d offset = Rotation2d.fromDegrees(88 - 90);
    public static final Rotation2d max = Rotation2d.fromDegrees(90);
    public static final Rotation2d min = Rotation2d.fromDegrees(10);
    public static final Rotation2d tolernace = Rotation2d.fromDegrees(1);

    public static final Rotation2d maxVelocityPerSecond = Rotation2d.fromDegrees(100);
    public static final Rotation2d maxAcceleration = Rotation2d.fromDegrees(120);

    public static double[] armSGV = new double[] { 0.01, 0.0425, 0.0 };
    public static double[] armPID = new double[] { 2, 1, 0f };

    // Interpolation
    // {meters, angle from horizontal}
    public static final Interpolation1D armAngleInterpolation = new Interpolation1D(
        new double[] { 1f, Units.degreesToRadians(10f) },
        new double[] { 10f, Units.degreesToRadians(30f) });
  }


  public static final class ClimberConstants{

    public static final int leftMotorID = 51;
    public static final int rightMotorID = 52;

    public static final double[] pid = {0,0,0};

   public static final double maxVelocity = 2000; // RPM/s
    public static final double maxAcceleration = 700; // RPM/s^2
    public static final double retractSpeed = 0.3;

    public static final TrapezoidProfile.Constraints climberConstraints = new TrapezoidProfile.Constraints(
        maxVelocity, maxAcceleration);
  }
}
