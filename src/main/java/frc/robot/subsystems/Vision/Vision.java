package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// https://github.com/themilkenknights/2024-Cresendo/blob/e38638b280864d1b3021e86e8846eaac7c18d05d/Cresendo/src/main/java/frc/robot/LimelightHelpers.java
// https://github.com/Roth-Shelley/SwerveKarma3/blob/master/src/main/java/frc/robot/subsystems/VisionSubsystem.java
public class Vision extends SubsystemBase {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public PhotonCamera camera = null;

    private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    public PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    public Vision() {
        camera = new PhotonCamera("cameraName"); // lime light
    }
}