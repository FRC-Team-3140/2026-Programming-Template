package frc.robot.subsystems.drivetrain;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  public static AprilTagFieldLayout fieldAprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  private VisionSystemSim visionSim;

  // TODO: Add Cameras with correct offsets
  public Camera[] cameras = new Camera[] {
    new Camera("Left", new Transform3d(
          Units.inchesToMeters(-10), 
          Units.inchesToMeters(10), 
          Units.inchesToMeters(10), 
          new Rotation3d(
            Units.degreesToRadians(0), 
            Units.degreesToRadians(10), 
            Units.degreesToRadians(180-45)))),

    new Camera("Right", new Transform3d(
          Units.inchesToMeters(-10), 
          Units.inchesToMeters(-10), 
          Units.inchesToMeters(10), 
          new Rotation3d(
            Units.degreesToRadians(0), 
            Units.degreesToRadians(10), 
            Units.degreesToRadians(180+45)))),
  };


  public Vision() {
    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(fieldAprilTags);

      for (Camera camera : cameras) {
        visionSim.addCamera(camera.getSimCamera(), camera.getOffset());
      }
    }
  }

  @Override
  public void periodic() {
    for (Camera camera : cameras) camera.update();
    if (Robot.isSimulation()) visionSim.update(RobotContainer.swerveDrive.getSimPose());
  }

  public static class Camera {
    private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, Double.MAX_VALUE);
    private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    private Matrix<N3, N1> curStdDevs = singleTagStdDevs;

    private int xRes = 1080;
    private int yRes = 720;
    private double fov = 120;
    private double fps = 90;

    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private PhotonPoseEstimator estimator;

    private Transform3d offset;

    public Camera(String name, Transform3d offset) {
      camera = new PhotonCamera(name);
      estimator = new PhotonPoseEstimator(fieldAprilTags, offset);

      this.offset = offset;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(xRes, yRes, Rotation2d.fromDegrees(fov));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(fps);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);

        simCamera = new PhotonCameraSim(camera, cameraProp);

        simCamera.enableDrawWireframe(true);
      }
    }

    public void update() {
      for (var result : camera.getAllUnreadResults()) {
        Optional<EstimatedRobotPose> visionEst = estimate(result);
        updateEstimationStdDevs(visionEst, result.getTargets());

        if (visionEst.isEmpty()) continue;

        EstimatedRobotPose est = visionEst.get();
        RobotContainer.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, curStdDevs);
      }
    }

    private Optional<EstimatedRobotPose> estimate(PhotonPipelineResult result) {
      Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = estimator.estimateLowestAmbiguityPose(result);
      }
      return visionEst;
    }

    public PhotonCameraSim getSimCamera() {
      return simCamera;
    }

    public Transform3d getOffset() {
      return offset;
    }

    /// PHOTONVISION PROVIDED 
    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
          var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;

          numTags++;
          avgDist +=
            tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = multiTagStdDevs;
          // Increase std devs based on (average) distance
          // max distance 15 meters
          if (numTags == 1 && avgDist > 15) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else { 
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}
