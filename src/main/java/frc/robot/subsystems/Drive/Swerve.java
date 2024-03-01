package frc.robot.subsystems.Drive;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.DriveMode;

public class Swerve extends SubsystemBase {
  private SwerveModule[] mSwerveMods;
  private GyroPigeon2 gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

  public final VisionPhotonvision vision = new VisionPhotonvision();

  private final PIDController snapPIDController;
  private DriveMode driveMode = DriveMode.DriverInput;
  private Rotation2d snapSetpoint = new Rotation2d();

  private Field2d debugField2d = new Field2d();

  // Tunable values
  private LoggedTunableNumber driveP = new LoggedTunableNumber("driveP", Constants.SwerveConstants.drivePID[0]);
  private LoggedTunableNumber driveI = new LoggedTunableNumber("driveI", Constants.SwerveConstants.drivePID[1]);
  private LoggedTunableNumber driveD = new LoggedTunableNumber("driveD", Constants.SwerveConstants.drivePID[2]);

  private LoggedTunableNumber driveS = new LoggedTunableNumber("driveS", Constants.SwerveConstants.driveSVA[0]);
  private LoggedTunableNumber driveV = new LoggedTunableNumber("driveV", Constants.SwerveConstants.driveSVA[1]);
  private LoggedTunableNumber driveA = new LoggedTunableNumber("driveA", Constants.SwerveConstants.driveSVA[2]);

  private LoggedTunableNumber angleP = new LoggedTunableNumber("angleP", Constants.SwerveConstants.anglePID[0]);
  private LoggedTunableNumber angleI = new LoggedTunableNumber("angleI", Constants.SwerveConstants.anglePID[1]);
  private LoggedTunableNumber angleD = new LoggedTunableNumber("angleD", Constants.SwerveConstants.anglePID[2]);

  private LoggedTunableNumber snapP = new LoggedTunableNumber("SnapP", Constants.SwerveConstants.snapPID[0]);
  private LoggedTunableNumber snapI = new LoggedTunableNumber("SnapI", Constants.SwerveConstants.snapPID[1]);
  private LoggedTunableNumber snapD = new LoggedTunableNumber("SnapD", Constants.SwerveConstants.snapPID[2]);

  public Swerve() {
    /* Gyro setup */
    gyro = new GyroPigeon2(Constants.SwerveConstants.pigeonID);
    gyro.home();

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(),
        new Pose2d());

    snapPIDController = new PIDController(
        Constants.SwerveConstants.snapPID[0],
        Constants.SwerveConstants.snapPID[1],
        Constants.SwerveConstants.snapPID[2]);

    snapPIDController.setTolerance(Units.degreesToRadians(1));
    snapPIDController.enableContinuousInput(0, Math.PI * 2);
    AutoBuilder.configureHolonomic(this::getEstimatedPose, this::setPose, this::getRobotRelativeSpeeds,
        this::driveRobotRelative, new HolonomicPathFollowerConfig(
            Constants.AutoConstants.translationPID, // Translation constants
            Constants.AutoConstants.rotationPID, // Rotation constants
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.SwerveConstants.wheelBase / 2, // Drive base radius (distance from center to furthest module)
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this);

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          debugField2d.getObject("pathplanner target pose").setPose(targetPose);
        });

    SmartDashboard.putData("Vision field", debugField2d);
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (driveMode == DriveMode.Snap) {
      return Optional.of(snapSetpoint);
    } else {
      return Optional.empty();
    }
  }

  public void setSnapSetpoint(Rotation2d setpoint) {
    this.snapSetpoint = setpoint;
  }

  public boolean isSnapAtSetpoint() {
    return snapPIDController.atSetpoint();
  }

  public Rotation2d getYawForSnap() {
    double yawRad = getYaw().getRadians();
    yawRad = yawRad % (Math.PI * 2);
    if (yawRad < 0) {
      yawRad = Math.PI * 2 + yawRad;
    }

    return new Rotation2d(yawRad);
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   */
  public void drive(Translation2d translation, double rotation) {
    switch (driveMode) {
      case Snap:
        rotation = -snapPIDController.calculate(getYawForSnap().getRadians(), snapSetpoint.getRadians());
        rotation = MathUtil.clamp(rotation, -1, 1);
        break;
      case DriverInput:

        break;
      default:
        break;
    }
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX() * Constants.SwerveConstants.maxSpeed,
            translation.getY() * Constants.SwerveConstants.maxSpeed,
            rotation * Constants.SwerveConstants.maxAngularVelocity, getYaw()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    setStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond,
        -targetSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] states) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber]);
    }
  }

  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  /**
   * Get the pitch from gyro
   * 
   * @return the pitch of the robot
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(gyro.getRoll());
  }

  public void zeroGyro() {
    gyro.homeYaw();
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (angleP.hasChanged() || angleI.hasChanged() || angleD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.angleController.setP(angleP.get());
        mod.angleController.setI(angleI.get());
        mod.angleController.setD(angleD.get());
      }
    }

    if (driveP.hasChanged() || driveI.hasChanged() || driveD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.driveController.setP(driveP.get());
        mod.driveController.setI(driveI.get());
        mod.driveController.setD(driveD.get());
      }
    }
    if (driveS.hasChanged() || driveV.hasChanged() || driveA.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.feedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
      }
    }

    if (snapP.hasChanged() || snapI.hasChanged() || snapD.hasChanged()) {
      snapPIDController.setPID(snapP.get(), snapI.get(), snapD.get());
    }
  }

  public void updateVisionMeasurements() {
    var visionEst = vision.getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      var estPose = est.estimatedPose.toPose2d();
      var estStdDevs = vision.getEstimationStdDevs(estPose);
      poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  }

  private Optional<Pose3d> getSpeakerPose() {
    var alliance = DriverStation.getAlliance();
    Optional<Pose3d> ampPose = null;

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        ampPose = vision.kTagLayout.getTagPose(7);
      } else {
        ampPose = vision.kTagLayout.getTagPose(4);
      }
    } else {
      ampPose = vision.kTagLayout.getTagPose(7);
    }

    return ampPose;
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    vision.resetPose(pose);
  }

  public double getDistanceFromSpeaker() {
    return getEstimatedPose().getTranslation().getDistance(getSpeakerPose().get().getTranslation().toTranslation2d());
  }

  public Rotation2d getRotationRelativeToSpeaker() {
    return getEstimatedPose().getTranslation().minus(getSpeakerPose().get().getTranslation().toTranslation2d())
        .unaryMinus()
        .getAngle();
  }

  public Optional<PhotonTrackedTarget> getClosestTag() {
    var targets = vision.getLatestResult().getTargets();
    Optional<PhotonTrackedTarget> result = Optional.empty();

    for (var target : targets) {
      if (target.getPoseAmbiguity() > 0.3) {
        break;
      }
      if (result.isPresent()) {
        if (result.get().getBestCameraToTarget().getTranslation().getNorm() > target.getBestCameraToTarget()
            .getTranslation().getNorm())
          result = Optional.of(target);
      } else {
        result = Optional.of(target);
      }
    }

    return result;
  }

  public void logValues() {
    SmartDashboard.putNumber("Distance From Speaker", getDistanceFromSpeaker());
    SmartDashboard.putNumber("Rotation to Speaker", getRotationRelativeToSpeaker().getDegrees());

    debugField2d.setRobotPose(getEstimatedPose());
    if (vision.latestVision.isPresent()) {
      debugField2d.getObject("Vision Intrusive Thoughts").setPose(vision.latestVision.get().estimatedPose.toPose2d());
    }

    SmartDashboard.putNumber("Snap Setpoint", snapSetpoint.getDegrees());
    SmartDashboard.putNumber("Snap Actual", getYawForSnap().getDegrees());

  }

  public void burnToFlash() {
    for (SwerveModule mod : mSwerveMods) {
      mod.burnToFlash();
    }
  }

  @Override
  public void periodic() {
    updateVisionMeasurements();
    checkTunableValues();
    for (SwerveModule mod : mSwerveMods) {
      mod.logValues();
    }
    gyro.logValues();

    poseEstimator.update(getYaw(), getPositions());
    logValues();
  }
}
