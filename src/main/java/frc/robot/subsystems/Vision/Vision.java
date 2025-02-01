package frc.robot.subsystems.Vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {

    public Vision() {
        LimelightHelpers.setPipelineIndex("", 0);
        // Basic targeting data
        LimelightHelpers.setLEDMode_ForceOn("");
    }

    public LimelightAprilTag closestTag =  new LimelightAprilTag();
    public AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    // public List<Optional<Pose3d>> coralTags = Arrays.asList(
    // fieldLayout.getTagPose(17),
    // fieldLayout.getTagPose(18),
    // fieldLayout.getTagPose(19),
    // fieldLayout.getTagPose(20),
    // fieldLayout.getTagPose(21),
    // fieldLayout.getTagPose(22)
    // );

    public List<AprilTag> tags = fieldLayout.getTags();

    class LimelightAprilTag {
        AprilTag tag;
        public int id;
        public double distFromCamera;
        public double xOffset;

        public LimelightAprilTag() {
            
        }
    }

    @Override
    public void periodic() {

        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id; // Tag ID
            double txnc = fiducial.txnc; // X offset (no crosshair)
            double tync = fiducial.tync; // Y offset (no crosshair)
            double ta = fiducial.ta; // Target area
            double distToCamera = fiducial.distToCamera; // Distance to camera
            double distToRobot = fiducial.distToRobot; // Distance to robot
            double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
            System.out.println("ID: " + id + " DIST: " + distToCamera);

            closestTag.id = id;
            closestTag.xOffset = txnc;
            closestTag.distFromCamera = distToCamera;
        }

        // LimelightResults results = LimelightHelpers.getLatestResults("");

        // if (results.valid) {
        // System.out.println("FOUND A APRIL TAG");
        // // AprilTags/Fiducials
        // if (results.targets_Fiducials.length > 0) {
        // LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
        // double id = tag.fiducialID; // Tag ID
        // System.out.println("FOUND TAG: "+id);
        // String family = tag.fiducialFamily; // Tag family (e.g., "16h5")

        // // 3D Pose Data
        // Pose3d robotPoseField = tag.getRobotPose_FieldSpace(); // Robot's pose in
        // field space
        // Pose3d cameraPoseTag = tag.getCameraPose_TargetSpace(); // Camera's pose
        // relative to tag
        // Pose3d robotPoseTag = tag.getRobotPose_TargetSpace(); // Robot's pose
        // relative to tag
        // Pose3d tagPoseCamera = tag.getTargetPose_CameraSpace(); // Tag's pose
        // relative to camera
        // Pose3d tagPoseRobot = tag.getTargetPose_RobotSpace(); // Tag's pose relative
        // to robot

        // // 2D targeting data
        // double tx = tag.tx; // Horizontal offset from crosshair
        // double ty = tag.ty; // Vertical offset from crosshair
        // double ta = tag.ta; // Target area (0-100% of image)
        // }
        // }
    }
}
