package frc.robot.subsystems.Drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

        SmartDashboard.putNumber("gyroInitReading", gyro.getAngle());

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
                        Constants.Swerve.FrontLeftModule.driveMotorID,
                        Constants.Swerve.FrontLeftModule.rotationMotorID,
                        Constants.Swerve.FrontLeftModule.canCoderID,
                        Constants.Swerve.FrontLeftModule.driveInvert,
                        Constants.Swerve.FrontLeftModule.rotationInvert,
                        Constants.Swerve.FrontLeftModule.canCoderInvert,
                        Constants.Swerve.FrontLeftModule.angleOffset)),
                new SwerveModule(1, new ModuleConfig(
                        Constants.Swerve.FrontRightModule.driveMotorID,
                        Constants.Swerve.FrontRightModule.rotationMotorID,
                        Constants.Swerve.FrontRightModule.canCoderID,
                        Constants.Swerve.FrontRightModule.driveInvert,
                        Constants.Swerve.FrontRightModule.rotationInvert,
                        Constants.Swerve.FrontRightModule.canCoderInvert,
                        Constants.Swerve.FrontRightModule.angleOffset)),
                new SwerveModule(2, new ModuleConfig(
                        Constants.Swerve.BackLeftModule.driveMotorID,
                        Constants.Swerve.BackLeftModule.rotationMotorID,
                        Constants.Swerve.BackLeftModule.canCoderID,
                        Constants.Swerve.BackLeftModule.driveInvert,
                        Constants.Swerve.BackLeftModule.rotationInvert,
                        Constants.Swerve.BackLeftModule.canCoderInvert,
                        Constants.Swerve.BackLeftModule.angleOffset)),
                new SwerveModule(3, new ModuleConfig(
                        Constants.Swerve.BackRightModule.driveMotorID,
                        Constants.Swerve.BackRightModule.rotationMotorID,
                        Constants.Swerve.BackRightModule.canCoderID,
                        Constants.Swerve.BackRightModule.driveInvert,
                        Constants.Swerve.BackRightModule.rotationInvert,
                        Constants.Swerve.BackRightModule.canCoderInvert,
                        Constants.Swerve.BackRightModule.angleOffset))
        };

        estimateOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            gyro.getRotation2d(),
            getModulePositions(),
            new Pose2d(new Translation2d(-8.175 + 0.45, 1.4478), new Rotation2d(180)),
            stateStdDevs,
            visionMeasurementStdDevs);
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
        if(Robot.isSimulation()) {
            return estimateOdometry.getEstimatedPosition();
        } else {
            return swerveOdometry.getPoseMeters();
        }
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[mSwerveMods.length];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = new SwerveModulePosition(mSwerveMods[i].getPosition(),mSwerveMods[i].getAngle());
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.Gyroscope.convention == gyroscope_convention.RIGHT_IS_POSITIVE)
                ? Rotation2d.fromDegrees(-gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        estimateOdometry.update(getYaw(), getModulePositions());

        SmartDashboard.putData("field", field);

        SmartDashboard.putNumber("localizationX", estimateOdometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("localizationY", estimateOdometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("localizationR", gyro.getAngle());
        SmartDashboard.putNumber("Gyrothingy Displacement", gyro.getDisplacementY());
        SmartDashboard.putNumber("odometryX", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("odometryY", swerveOdometry.getPoseMeters().getY());
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        field.setRobotPose(CoordinateSystems.FieldMiddle_FieldBottomLeft(new Pose2d(estimateOdometry.getEstimatedPosition().getX(), estimateOdometry.getEstimatedPosition().getY(), gyro.getRotation2d())));

        if(vision.camera != null) {
            Optional<EstimatedRobotPose> estimatedPoseOpt = vision.photonPoseEstimator.update(vision.camera.getLatestResult());
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