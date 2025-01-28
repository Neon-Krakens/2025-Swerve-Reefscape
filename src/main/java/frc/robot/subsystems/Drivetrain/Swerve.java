package frc.robot.subsystems.Drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.math.CoordinateSystems;
import frc.robot.Constants.Swerve.gyroscope_convention;
import frc.robot.subsystems.Drivetrain.SwerveModuleConfig.ModuleConfig;
import frc.robot.subsystems.Vision.Vision;

/*
 * Modified from https://github.com/DIEHDZ/Swerve-base-Crescendo-2024/blob/master/src/main/java/frc/robot/subsystems/Drivetrain/Swerve.java
 */
public class Swerve extends SubsystemBase {
    private final AHRS gyro = Constants.Swerve.Gyroscope.gyro;
    private final Field2d field;
    private final SwerveDrivePoseEstimator estimateOdometry;
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] mSwerveMods;

    // Standard deviations for the state estimate
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
    private Vision vision;

    public Swerve(Vision vision) {
        zeroGyro();
        field = new Field2d();
        this.vision = vision;

        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.swerveKinematics,
                getYaw(),
                new SwerveModulePosition[] { // Initialize with zeros
                        new SwerveModulePosition(0, new Rotation2d(0)),
                        new SwerveModulePosition(0, new Rotation2d(0)),
                        new SwerveModulePosition(0, new Rotation2d(0)),
                        new SwerveModulePosition(0, new Rotation2d(0))
                });

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, new ModuleConfig(
                        Constants.Swerve.FL.driveMotorID,
                        Constants.Swerve.FL.rotationMotorID,
                        Constants.Swerve.FL.canCoderID,
                        Constants.Swerve.FL.driveInvert,
                        Constants.Swerve.FL.rotationInvert,
                        Constants.Swerve.FL.canCoderInvert,
                        Constants.Swerve.FL.angleOffset)),
                new SwerveModule(1, new ModuleConfig(
                        Constants.Swerve.FR.driveMotorID,
                        Constants.Swerve.FR.rotationMotorID,
                        Constants.Swerve.FR.canCoderID,
                        Constants.Swerve.FR.driveInvert,
                        Constants.Swerve.FR.rotationInvert,
                        Constants.Swerve.FR.canCoderInvert,
                        Constants.Swerve.FR.angleOffset)),
                new SwerveModule(2, new ModuleConfig(
                        Constants.Swerve.BL.driveMotorID,
                        Constants.Swerve.BL.rotationMotorID,
                        Constants.Swerve.BL.canCoderID,
                        Constants.Swerve.BL.driveInvert,
                        Constants.Swerve.BL.rotationInvert,
                        Constants.Swerve.BL.canCoderInvert,
                        Constants.Swerve.BL.angleOffset)),
                new SwerveModule(3, new ModuleConfig(
                        Constants.Swerve.BR.driveMotorID,
                        Constants.Swerve.BR.rotationMotorID,
                        Constants.Swerve.BR.canCoderID,
                        Constants.Swerve.BR.driveInvert,
                        Constants.Swerve.BR.rotationInvert,
                        Constants.Swerve.BR.canCoderInvert,
                        Constants.Swerve.BR.angleOffset))
        };

        estimateOdometry = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                gyro.getRotation2d(),
                getModulePositions(),
                new Pose2d(new Translation2d(-8.175 + 0.45, 1.4478), new Rotation2d(180)),
                stateStdDevs,
                visionMeasurementStdDevs);

        zeroWheels();
        configurePathplanner();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public Pose2d getPose() {
        if (Robot.isSimulation()) {
            return estimateOdometry.getEstimatedPosition();
        } else {
            return swerveOdometry.getPoseMeters();
        }
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    private void configurePathplanner() {
        // RobotConfig config = null;
        // try {
        //     config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     // Handle exception as needed
        //     e.printStackTrace();
        // }

        // // Configure AutoBuilder last
        // AutoBuilder.configure(
        //         this::getPose, // Robot pose supplier
        //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
        //                                                               // RELATIVE ChassisSpeeds. Also optionally outputs
        //                                                               // individual module feedforwards
        //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
        //                                         // holonomic drive trains
        //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //         ),
        //         config, // The robot configuration
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red
        //             // alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        // );
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModulesStates());
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        setModulesStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void setModulesStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        mSwerveMods[Constants.Swerve.FL.moduleId].setDesiredState(states[Constants.Swerve.FL.moduleId], true);
        mSwerveMods[Constants.Swerve.FR.moduleId].setDesiredState(states[Constants.Swerve.FR.moduleId], true);
        mSwerveMods[Constants.Swerve.BL.moduleId].setDesiredState(states[Constants.Swerve.BL.moduleId], true);
        mSwerveMods[Constants.Swerve.BR.moduleId].setDesiredState(states[Constants.Swerve.BR.moduleId], true);
    }

    public SwerveModuleState[] getModulesStates() {
        return new SwerveModuleState[] {
                mSwerveMods[0].getState(),
                mSwerveMods[1].getState(),
                mSwerveMods[2].getState(),
                mSwerveMods[3].getState(),
        };
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = new SwerveModulePosition(mSwerveMods[i].getPosition(), mSwerveMods[i].getAngle());
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public void zeroWheels() {
        for (SwerveModule mod : mSwerveMods) {
            mod.setAngle(0);
        }
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.Gyroscope.convention == gyroscope_convention.RIGHT_IS_POSITIVE)
                ? Rotation2d.fromDegrees(-gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }


    public void setAllWheelDirection(double angle) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setAngle(angle);
        }
    }

    public void setAllWheelSpeed(double speed) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setDriveSpeed(speed);
        }
    }

    // 0-360
    public void setBotDirection(double targetAngle) {
        double currentHeading = getYaw().getDegrees();
        double deltaAngle = targetAngle - currentHeading;

        // if(deltaAngle < -180) {
        //     deltaAngle += 180;
        // }

        // Normalize deltaAngle to be within [-180, 180]
        System.out.println(deltaAngle+" "+ targetAngle);
        mSwerveMods[0].setAngle(45+180); 
        mSwerveMods[1].setAngle(45+180+90); 
        mSwerveMods[2].setAngle(45+90); 
        mSwerveMods[3].setAngle(45); 
            
        if(deltaAngle < 0) {
            setAllWheelSpeed(0.05);
        } else {
            setAllWheelSpeed(-0.05);
        }

        // System.out.println(currentHeading+" "+targetAngle);

        // Determine the direction to rotate the wheels
        // // if (...) {
        

        // Set the rotation speed based on the deltaAngle
        double rotationSpeed = deltaAngle / 180.0; // Scale to [-1, 1]
        rotationSpeed = Math.max(-Constants.Swerve.maxWheelRotateSpeed, Math.min(Constants.Swerve.maxWheelRotateSpeed, rotationSpeed));

        // Set the wheel speeds for rotation
    }


    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        estimateOdometry.update(getYaw(), getModulePositions());

        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putData("field", field);

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees()*180);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond);
        }
        field.setRobotPose(
                CoordinateSystems.FieldMiddle_FieldBottomLeft(new Pose2d(estimateOdometry.getEstimatedPosition().getX(),
                        estimateOdometry.getEstimatedPosition().getY(), gyro.getRotation2d())));

        if (vision.camera != null) {
            Optional<EstimatedRobotPose> estimatedPoseOpt = vision.photonPoseEstimator
                    .update(vision.camera.getLatestResult());
            if (estimatedPoseOpt.isPresent()) {
                EstimatedRobotPose pose = estimatedPoseOpt.get();
                estimateOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
            }
        }
    }

    // From
    // https://github.com/Frc5572/Jetsi-Offseason/blob/26e2ee780a82ebb3c5df38d7c8fb2befd7773aef/src/main/java/frc/lib/util/swerve/SwerveModuleReal.java#L137
    public static void configureCANcoder(CANcoder canCoder, boolean isInverted) {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        canCoderConfig.MagnetSensor.SensorDirection = isInverted ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        canCoder.getConfigurator().apply(canCoderConfig);
    }

}