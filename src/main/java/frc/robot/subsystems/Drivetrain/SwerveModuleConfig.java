package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

/*
 * Taken from https://github.com/DIEHDZ/Swerve-base-Crescendo-2024/blob/master/src/main/java/frc/lib/config/SwerveModuleConfig.java
 */
public class SwerveModuleConfig {
    public static class ModuleConfig {
        public final int driveMotorID;
        public final int rotationMotorID;
        public final int canCoderID;
        public final boolean driveInvert;
        public final boolean rotationInvert;
        public final boolean canCoderInvert;
        public final Rotation2d angleOffset;

        public ModuleConfig(int driveMotorID, int rotationMotorID, int canCoderID,
                boolean driveInvert, boolean rotationInvert, boolean canCoderInvert,
                Rotation2d angleOffset) {
            this.driveMotorID = driveMotorID;
            this.rotationMotorID = rotationMotorID;
            this.canCoderID = canCoderID;
            this.driveInvert = driveInvert;
            this.rotationInvert = rotationInvert;
            this.canCoderInvert = canCoderInvert;
            this.angleOffset = angleOffset;
        }
    }
}