package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Swerve {

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(10.875 * 2);
        public static final double wheelBase = Units.inchesToMeters(10.875 * 2);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.12;
        public static final double angleGearRatio = 150/7;

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        
        //Swerve Voltage Compensation
        public static final double voltafeComp = 12.0;

        // Swerve Current Limiting
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 80;

        //Deadband
        public static final double stickDeadband = 0.05; 

        // rotation motor PID values
        public static final double rotationKP = 0.01;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;
        public static final double rotationKFF = 0.0;

        // drive motor PID values
        public static final double drivelKP = 0.01;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        // Drive Motor Conversion Factors
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

        // Angle Motor Conversion Factors
        public static final double angleConversionPositionFactor = wheelCircumference / angleGearRatio;
        public static final double angleConversionVelocityFactor = angleConversionPositionFactor / 60.0;

        // Swerve Profiling Values
        public static final double maxSpeed = 5; // meters per second
        public static final double maxAngularVelocity = 10;

        // Neutral Modes
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        // Gyroscope Constants
        public static final class Gyroscope {
            public static final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
            public static final gyroscope_convention convention = gyroscope_convention.RIGHT_IS_POSITIVE;
        } 

        // Gyroscope convention enumeration
        public enum gyroscope_convention {
            LEFT_IS_POSITIVE(1),
            RIGHT_IS_POSITIVE(-1);

            public final int value;

            gyroscope_convention(int value) {
                this.value = value;
            }
        }

        // Swerve Modules

        // Front Left Module - Module 0
        public static final class FrontLeftModule {
            public static final int driveMotorID = 1;
            public static final int rotationMotorID = 2;
            public static final int canCoderID = 13;
            public static final boolean driveInvert = false;
            public static final boolean rotationInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
        }

        // Front Right Module - Module 1
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int rotationMotorID = 4;
            public static final int canCoderID = 23;
            public static final boolean driveInvert = false;
            public static final boolean rotationInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
        }

        // Back Left Module - Module 2
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int rotationMotorID = 6;
            public static final int canCoderID = 33;
            public static final boolean driveInvert = false;
            public static final boolean rotationInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
        }

        // Back Right Module - Module 3
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int rotationMotorID = 8;
            public static final int canCoderID = 43;
            public static final boolean driveInvert = false;
            public static final boolean rotationInvert = false;
            public static final boolean canCoderInvert = false;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1);
        }
    }
}