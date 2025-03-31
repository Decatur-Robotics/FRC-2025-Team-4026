package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain.PathLocation;

public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain swerve;

    private final AprilTagFieldLayout aprilTagLayout;

    private final PhotonCamera cameraFrontLeft, cameraFrontRight; // , cameraBack;

    private final PhotonPoseEstimator poseEstimatorFrontLeft, poseEstimatorFrontRight; // , poseEstimatorBack;

   private DoublePublisher coralEstimator;

    private StructPublisher<Pose3d> publisherFrontLeft = NetworkTableInstance.getDefault()
        .getStructTopic("CameraFrontLeft", Pose3d.struct).publish();
    private StructPublisher<Pose3d> publisherFrontRight = NetworkTableInstance.getDefault()
        .getStructTopic("CameraFrontRight", Pose3d.struct).publish();
    // private StructPublisher<Pose3d> publisherBack = NetworkTableInstance.getDefault()
    //     .getStructTopic("CameraBack", Pose3d.struct).publish();
    
    public VisionSubsystem(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;

        aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        cameraFrontLeft = new PhotonCamera(VisionConstants.CAMERA_FRONT_LEFT_NAME);
        cameraFrontRight = new PhotonCamera(VisionConstants.CAMERA_FRONT_RIGHT_NAME);
        // cameraBack = new PhotonCamera(VisionConstants.CAMERA_BACK_NAME);

        poseEstimatorFrontLeft = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT);
        poseEstimatorFrontRight = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            VisionConstants.ROBOT_TO_CAMERA_FRONT_RIGHT);
        // poseEstimatorBack = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        //     VisionConstants.ROBOT_TO_CAMERA_FRONT_LEFT);
    }

    @Override
    public void periodic() {
        updateVisionMeasurements(cameraFrontLeft, poseEstimatorFrontLeft, publisherFrontLeft);
        updateVisionMeasurements(cameraFrontRight, poseEstimatorFrontRight, publisherFrontRight);
        // updateVisionMeasurements(cameraBack, poseEstimatorBack, publisherBack);
    }

    public void updateVisionMeasurements(PhotonCamera camera, PhotonPoseEstimator poseEstimator, 
            StructPublisher<Pose3d> publisher) {
        Optional<EstimatedRobotPose> poseEstimate = Optional.empty();
        Matrix<N3, N1> standardDeviations = VisionConstants.SINGLE_TAG_STANDARD_DEVIATIONS;

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!swerve.getTargetPoseLocation().equals(PathLocation.None)) {
                List<PhotonTrackedTarget> targets = result.getTargets();

                List<PhotonTrackedTarget> desiredTargets = new ArrayList<>();

                List<Integer> apriltagIds = swerve.getTargetPoseLocation().getApriltagIds();

                for (PhotonTrackedTarget target : targets) {
                    for (int i : apriltagIds) {
                        if (target.getFiducialId() == i) {
                            desiredTargets.add(target);
                        }
                    }
                }

                result = new PhotonPipelineResult(result.metadata, desiredTargets, Optional.empty());
            }

            poseEstimate = poseEstimator.update(result);
            standardDeviations = getStandardDeviations(poseEstimate, result.getTargets());

            if (poseEstimate.isPresent()) {
                swerve.addVisionMeasurement(poseEstimate.get().estimatedPose.toPose2d(), poseEstimate.get().timestampSeconds, standardDeviations);
                publisher.set(poseEstimate.get().estimatedPose);
            }
        }
    }

    public Matrix<N3, N1> getStandardDeviations(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return VisionConstants.SINGLE_TAG_STANDARD_DEVIATIONS;
        } 
        else {
            // Pose present. Start running Heuristic
            int numberTags = 0;
            double averageDistance = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var target : targets) {
                var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                
                if (tagPose.isEmpty()) continue;

                numberTags++;
                averageDistance += tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numberTags == 0) {
                // No tags visible. Default to single-tag std devs
                return VisionConstants.SINGLE_TAG_STANDARD_DEVIATIONS;
            } 
            else {
                Matrix<N3, N1> estimatedStandardDeviations = VisionConstants.SINGLE_TAG_STANDARD_DEVIATIONS;
                // One or more tags visible, run the full heuristic.
                averageDistance /= numberTags;

                // Decrease std devs if multiple targets are visible
                if (numberTags > 1) estimatedStandardDeviations = VisionConstants.MULTI_TAG_STANDARD_DEVIATIONS;

                // Increase std devs based on (average) distance
                if (numberTags == 1 && averageDistance > 4)
                    return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else return estimatedStandardDeviations.times(1 + (averageDistance * averageDistance / 30));
            }
        }
    }

}
