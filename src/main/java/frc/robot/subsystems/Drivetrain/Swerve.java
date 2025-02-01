package frc.robot.subsystems.Drivetrain;

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

    public Swerve() {
        zeroGyro();
        field = new Field2d();

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
            mod.setAngle(0,false);
        }
        zeroGyro();
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
            boolean gotToTarget = mod.setAngle(angle, true);
            if(!gotToTarget) {
                allWheelsRotatedToTarget = false;
            }
        }
    }

    public void setAllWheelRotationSpeed(double speed) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setRotationSpeed(speed);
        }
    }

    public void setAllWheelSpeed(double speed) {
        for (SwerveModule mod : mSwerveMods) {
            mod.setDriveSpeed(speed);
        }
    }

    boolean rotatingToTarget = false;
    boolean driving = false;
    double targetAngle = 0.0;
    double botTranslationDegrees = 181.0;
    double lastBotTranslationDegrees = 181.0;

    boolean allWheelsRotatedToTarget = false;

    double botTranslationSpeed = 0.0;

    public void update() {
        if(rotatingToTarget) {
            if(driving) return;

            mSwerveMods[0].setAngle(45+180,false);
            mSwerveMods[1].setAngle(45+180+90,false); 
            mSwerveMods[2].setAngle(45+90,false); 
            mSwerveMods[3].setAngle(45,false); 
                
            double currentHeading = -getYaw().getDegrees();
        
            double deltaAngle = targetAngle - currentHeading;
            double rotationSpeed = deltaAngle / -180.0; // Scale to [-1, 1]

            rotationSpeed = Math.max(-Constants.Swerve.maxWheelRotateSpeed, Math.min(Constants.Swerve.maxWheelRotateSpeed, rotationSpeed));
            if(deltaAngle < -5) {
                setAllWheelSpeed(-Math.abs(rotationSpeed));
            } else if(deltaAngle > 5) {
                setAllWheelSpeed(Math.abs(rotationSpeed));
            } else {
                rotatingToTarget = false;
                setAllWheelSpeed(0.0);
            }

            if(!rotatingToTarget) {
                driving = true;
                if(botTranslationDegrees != 181.0 && lastBotTranslationDegrees != botTranslationDegrees) {
                    allWheelsRotatedToTarget = true;
                    setAllWheelDirection(botTranslationDegrees);
                }
                lastBotTranslationDegrees = botTranslationDegrees;

                // Reset to rotation mode if No input on driving translation
                if(botTranslationSpeed == 0.0) {
                    setAllWheelRotationSpeed(0.0);
                    driving = false;
                }

                if(allWheelsRotatedToTarget) {
                    setAllWheelSpeed(botTranslationSpeed);
                } else {
                    setAllWheelSpeed(0.0);
                }
            }
        } else {
            driving = true;
            if(botTranslationDegrees != 181.0 && lastBotTranslationDegrees != botTranslationDegrees) {
                allWheelsRotatedToTarget = true;
                setAllWheelDirection(botTranslationDegrees);
            }
            lastBotTranslationDegrees = botTranslationDegrees;

            // Reset to rotation mode if No input on driving translation
            if(botTranslationSpeed == 0.0) {
                setAllWheelRotationSpeed(0.0);
                driving = false;
            }

            if(allWheelsRotatedToTarget) {
                setAllWheelSpeed(botTranslationSpeed);
            } else {
                setAllWheelSpeed(0.0);
            }
        }

        botTranslationSpeed = 0.0;
    }

    // 0-360
    public void setBotDirection(double targetAngle) {
        this.targetAngle = targetAngle;

        // Normalize deltaAngle to be within [-180, 180]
        rotatingToTarget = true;
    }

    public void rotateDrive(double strafeVal, double translationVal) {
        // Calculate the desired angle based on joystick X (strafe) and Y (translation)
        double desiredAngle = Math.atan2(strafeVal, translationVal);

        // Calculate the translation speed as the distance from (0, 0) to
        double translationSpeed = Math.sqrt(Math.pow(translationVal, 2) + Math.pow(strafeVal, 2));

        // Optionally, you can scale translationSpeed by maxSpeed if needed
        translationSpeed = translationSpeed * Constants.Swerve.maxSpeed;

        double angleDegrees = Math.toDegrees(desiredAngle) + getYaw().getDegrees();

        botTranslationSpeed = -translationSpeed;
        botTranslationDegrees = angleDegrees;
    }

    boolean zerod = false;
    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        estimateOdometry.update(getYaw(), getModulePositions());

        if(!zerod) {
            gyro.zeroYaw();
            zerod = true;
        }
        SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putData("field", field);

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees()*180);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Speed", mod.getState().speedMetersPerSecond);
        }
        field.setRobotPose(
                CoordinateSystems.FieldMiddle_FieldBottomLeft(new Pose2d(estimateOdometry.getEstimatedPosition().getX(),
                        estimateOdometry.getEstimatedPosition().getY(), gyro.getRotation2d())));
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