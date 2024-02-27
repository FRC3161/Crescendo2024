package frc.robot.subsystems.Drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class VisionPhotonvision {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private double lastEstTimestamp = 0;
  public Optional<EstimatedRobotPose> latestVision = Optional.empty();
  public AprilTagFieldLayout kTagLayout;

  public VisionPhotonvision() {
    kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    photonPoseEstimator = new PhotonPoseEstimator(kTagLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.VisionConstants.kRobotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult)
      lastEstTimestamp = latestTimestamp;

    latestVision = visionEst;
    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public void resetPose(Pose2d pose) {
    photonPoseEstimator.setLastPose(pose);
  }
}
