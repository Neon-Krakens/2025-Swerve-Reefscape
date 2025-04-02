package frc.robot.subsystems.Bot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveToPath;
import frc.robot.subsystems.Vision.Vision.Cameras;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;

import org.json.simple.parser.ParseException;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Manages the robot's swerve drive system, providing control over movement,
 * autonomous path following,
 * and odometry tracking. This subsystem handles both autonomous and
 * teleoperated drive control,
 * integrating with PathPlanner for advanced autonomous capabilities.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Field-oriented drive control
 * <li>Autonomous path following and path finding
 * <li>Odometry tracking and pose estimation
 * <li>Wheel locking for stability
 * </ul>
 * 
 * <p>
 * Uses YAGSL (Yet Another Generic Swerve Library) for underlying swerve drive
 * implementation
 * and PathPlanner for autonomous navigation.
 */

// TAKEN FROM
// https://github.com/4533-phoenix/frc-2025-robot/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
// https://docs.yagsl.com/configuring-yagsl/standard-conversion-factors
// https://docs.yagsl.com/configuring-yagsl/how-to-tune-pidf
public class Swerve extends SubsystemBase {
    private static Swerve instance;
    private final SwerveDrive swerveDrive;
    /**
     * Enable vision odometry updates while driving.
     */
    private final boolean visionDriveTest = true;
    /**
     * PhotonVision class to keep an accurate odometry.
     */
    private Vision vision;

    /**
     * Returns the singleton instance of the Swerve subsystem.
     * Creates a new instance if one does not exist.
     * 
     * @return the Swerve subsystem instance
     */
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    /**
     * Creates a new Swerve subsystem that manages drive control, path following,
     * and odometry.
     * Initializes the swerve drive with high telemetry verbosity and configures
     * various drive parameters.
     * 
     * @throws RuntimeException if swerve drive creation fails
     */
    public Swerve() {
        // SmartDashboard.putBoolean("Auto Align Left", false);
        // SmartDashboard.putBoolean("Auto Align Right", false);

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(
                        Constants.MAX_SPEED.in(MetersPerSecond),
                         new Pose2d(
                            new Translation2d(Meter.of(8.774),
                            Meter.of(4.026)),
                            Rotation2d.fromDegrees(0)
                        )
                    );
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive", e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
        swerveDrive.pushOffsetsToEncoders();
        swerveDrive.setMaximumAllowableSpeeds(Constants.MAX_SPEED.in(MetersPerSecond), Constants.MAX_ANGULAR_VELOCITY);

        if (visionDriveTest) {
            setupPhotonVision();
            // Stop the odometry thread if we are using vision that way we can synchronize
            // updates better.
            swerveDrive.stopOdometryThread();
        }
        setupPathPlanner();
        loadPaths();
    }

     // Path Planning Paths
    public PathPlannerPath Z1R;
    public PathPlannerPath Z1L;
    public PathPlannerPath Z2R;
    public PathPlannerPath Z2L;
    public PathPlannerPath Z3R;
    public PathPlannerPath Z3L;
    public PathPlannerPath Z4R;
    public PathPlannerPath Z4L;
    public PathPlannerPath Z5R;
    public PathPlannerPath Z5L;
    public PathPlannerPath Z6R;
    public PathPlannerPath Z6L;

    private void loadPaths() {
        try {
            System.out.println("\n[Path Loading] Loading paths...");
            Z1R = PathPlannerPath.fromPathFile("1R");
            Z1L = PathPlannerPath.fromPathFile("1L");
            Z2R = PathPlannerPath.fromPathFile("2R");
            Z2L = PathPlannerPath.fromPathFile("2L");
            Z3R = PathPlannerPath.fromPathFile("3R");
            Z3L = PathPlannerPath.fromPathFile("3L");
            Z4R = PathPlannerPath.fromPathFile("4R");
            Z4L = PathPlannerPath.fromPathFile("4L");
            Z5R = PathPlannerPath.fromPathFile("5R");
            Z5L = PathPlannerPath.fromPathFile("5L");
            Z6R = PathPlannerPath.fromPathFile("6R");
            Z6L = PathPlannerPath.fromPathFile("6L");
            System.out.println("-> All paths loaded successfully");
        } catch (FileVersionException | IOException | ParseException e) {
            System.err.println("!! ERROR LOADING PATHS !!");
            e.printStackTrace();
            // Handle null paths gracefully
            Z1R = null;
            Z1L = null;
            Z2R = null;
            Z2L = null;
            Z3R = null;
            Z3L = null;
            Z4R = null;
            Z4L = null;
            Z5R = null;
            Z5L = null;
            Z6R = null;
            Z6L = null;
        }
    }

    /**
     * Configures PathPlanner for autonomous path following.
     * Sets up the necessary callbacks and controllers for autonomous navigation,
     * including pose estimation, odometry reset, and velocity control.
     * Also initializes path finding warm-up for better initial performance.
     */
    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedForward = true;

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedForward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathfindingCommand.warmupCommand().schedule();

    }
    
    public Command goToClosestCoralTag(boolean alignLeftSide) {
        return run(() -> {
            Cameras camera = Cameras.CENTER_CAM;
            List<Pose2d> poses = new ArrayList<>();
            HashMap<Pose2d, Integer> tagMap = new HashMap<>();

            if(!DriverStation.getAlliance().isPresent()) {
                System.out.println("Alliance not present");
                return;
            }
            
            // Collect the valid coral tag poses
            camera.getFieldLayout().getTags().forEach(tag -> {
                Pose2d pose = tag.pose.toPose2d();
                boolean redTag = tag.ID == 6 || tag.ID == 7 || tag.ID == 8 || tag.ID == 9 || tag.ID == 10 || tag.ID == 11;
                boolean blueTag = tag.ID == 17 || tag.ID == 18 || tag.ID == 19 || tag.ID == 20 || tag.ID == 21 || tag.ID == 22;

                if(
                    (redTag && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) || 
                    (blueTag && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                ) {
                    poses.add(pose);
                    tagMap.put(pose, tag.ID);
                }
            });

            var closest = this.getPose().nearest(poses);
            var closestID = tagMap.get(closest);

            PathPlannerPath path = null;
            switch (closestID) {
                case 6: 
                case 19: 
                    path = alignLeftSide? Z6L : Z6R;
                    break;
                case 7: 
                case 18: 
                    path = alignLeftSide? Z1L : Z1R;
                    break;
                case 8: 
                case 17: 
                    path = alignLeftSide? Z2L : Z2R;
                    break;
                case 9: 
                case 22: 
                    path = alignLeftSide? Z3L : Z3R;
                    break;
                case 10: 
                case 21: 
                    path = alignLeftSide? Z4L : Z4R;
                    break;
                case 11: 
                case 20: 
                    path = alignLeftSide? Z5L : Z5R;
                    break;
                default:
                    break;
            }

            Command pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(
                Constants.MAX_SPEED.in(MetersPerSecond) * 0.5, // 50% of max velocity
                Constants.MAX_SPEED.in(MetersPerSecond) * 0.4, // 50% of max acceleration
                Constants.MAX_ANGULAR_VELOCITY * 0.7, // 70% of max angular velocity
                Constants.MAX_ANGULAR_VELOCITY * 0.7 // 70% of max angular acceleration
            ));
            swerveDrive.field.getObject("TAGS").setPoses(poses);

            pathFollowingCommand.schedule();
        });
    }

    public Command cancelPathfinding() {
        return run(() -> {
            this.getCurrentCommand().cancel();
        });
    }

    // public Command goToClosestDrop() {
    //     return run(() -> {
    //         Cameras camera = Cameras.CENTER_CAM;
    //         List<Pose2d> poses = new ArrayList<>();
            
    //         // Collect the valid depo tag poses
    //         camera.getFieldLayout().getTags().forEach(tag -> {
    //             Pose2d pose = tag.pose.toPose2d();
    //             boolean redTag = tag.ID == 2 || tag.ID == 1;
    //             boolean blueTag = tag.ID == 12 || tag.ID == 13;

    //             if(
    //                 (redTag && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) || 
    //                 (blueTag && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
    //             ) {
    //                 pose = pose.rotateAround(pose.getTranslation(), Rotation2d.fromDegrees(180)); 

    //                 Rotation2d rotation = pose.getRotation();
    //                 double angle = rotation.getRadians();

    //                 // Calculate the change in position
    //                 double deltaX = (-0.5) * Math.cos(angle);
    //                 double deltaY = (-0.5) * Math.sin(angle);
    //                 // Pose lined up with target
    //                 Pose2d newPose = new Pose2d(pose.getX() + deltaX, pose.getY() + deltaY, pose.getRotation());

    //                 poses.add(newPose);
    //             }
    //         });
    //         swerveDrive.field.getObject("TAGS").setPoses(poses);
    
    //         // Find the closest depo tag
    //         Pose2d closestDepoTag = this.getPose().nearest(poses);
    //         closestDepoTag = closestDepoTag.rotateAround(closestDepoTag.getTranslation(), Rotation2d.fromDegrees(-90));
    //         // Schedule the robot to move to the new adjusted pose
    //         CommandScheduler.getInstance().schedule(driveToPose(closestDepoTag));
    //     });
    // }

    public Command scootLeft() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(0, 0.5), 0, false, false);
        }); // Adjust time as needed
    }

    public Command scootRight() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(0, -0.5), 0, false, false);
        }); // Adjust time as needed
    }

    public Command scootBackward() {
        return runOnce(() -> {
            swerveDrive.drive(new Translation2d(-0.5, 0.0), 0, false, false);
        }); // Adjust time as needed
    }


    /**
     * Setup the photon vision class.
     */
    public void setupPhotonVision() {
        System.out.println("Setting Up Photon Vision");
        vision = new Vision(swerveDrive::getPose, swerveDrive.field);
    }

    @Override
    public void periodic() {
        // if(SmartDashboard.getBoolean("Auto Align Left", false)) {
        //     SmartDashboard.putBoolean("Auto Align Left", false);
            
        //     CommandScheduler.getInstance().schedule(goToClosestCoralTag(true));
        // }
        // if(SmartDashboard.getBoolean("Auto Align Right", false)) {
        //     SmartDashboard.putBoolean("Auto Align Right", false);
        //     CommandScheduler.getInstance().schedule(goToClosestCoralTag(false));
        // }
        // When vision is enabled we must manually update odometry in SwerveDrive
        if (visionDriveTest) {
            swerveDrive.updateOdometry();
            vision.updatePoseEstimation(swerveDrive);
            vision.updateVisionField();
            SmartDashboard.putNumber("GYRO", swerveDrive.getYaw().getDegrees());
        }
    }

    /**
     * Creates a command to drive the robot in field-oriented mode.
     * Takes into account the robot's current heading to maintain consistent
     * field-relative movement.
     * 
     * @param velocity a supplier that provides the desired chassis speeds in
     *                 field-oriented coordinates
     * @return a command that continuously updates drive output based on supplied
     *         velocities
     * @see ChassisSpeeds
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
    }

    /**
     * Locks the swerve modules in an X pattern to prevent the robot from moving.
     * Useful for maintaining position or preparing for disable.
     */
    public void lockWheels() {
        swerveDrive.lockPose();
    }

    /**
     * Resets the robot's odometry to the center of the field (8.774m, 4.026m, 0°).
     * This is typically used at the start of autonomous routines.
     */
    public void resetOdometry() {
        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),Rotation2d.fromDegrees(0)));
    }

    /**
     * Retrieves the current estimated pose of the robot on the field.
     * 
     * @return the current Pose2d representing the robot's position and rotation
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Retrieves the current estimated pose of the robot on the field.
     * 
     * @return the current Pose2d representing the robot's position and rotation
     */
    public Pose3d getPose3d() {
        Pose2d pose2d = swerveDrive.getPose();
        // Convert to Pose3d (setting z = 0 and using Rotation3d)
        Pose3d pose3d = new Pose3d(
            pose2d.getX(),
            pose2d.getY(),
            0.0, // Default z height
            new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()) // Convert Rotation2d to Rotation3d
        );

        return pose3d;
    }

    /**
     * Resets the robot's odometry to a specific pose.
     * 
     * @param pose the Pose2d to set as the robot's current position and rotation
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Gets the current velocity of the robot.
     * 
     * @return the ChassisSpeeds representing the robot's current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the underlying SwerveDrive object.
     * 
     * @return the SwerveDrive instance used by this subsystem
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Creates a command to autonomously drive the robot to a specific pose using
     * PathPlanner.
     * 
     * @param pose the target Pose2d to drive to
     * @return a Command that will drive the robot to the specified pose
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                Constants.MAX_ANGULAR_VELOCITY, 
                Constants.MAX_ANGULAR_VELOCITY);
                
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }
    

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                Constants.MAX_SPEED2);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }
}