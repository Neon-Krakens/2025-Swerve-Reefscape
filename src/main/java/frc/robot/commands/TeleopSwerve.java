package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
        private final Swerve s_Swerve;
        private final DoubleSupplier translationSup;
        private final DoubleSupplier strafeSup;
        private final DoubleSupplier rotationXSup;
        private final DoubleSupplier rotationYSup;

        private final BooleanSupplier toggleFieldOriented;

        private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

        private boolean isFieldOriented = true; // Field oriented means the robot drives relative to the field, not the
                                                // robot

        public TeleopSwerve(
                        Swerve s_Swerve,
                        DoubleSupplier translationSup,
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSupX,
                        DoubleSupplier rotationSupY,
                        BooleanSupplier toggleFieldOriented) {
                this.s_Swerve = s_Swerve;
                addRequirements(s_Swerve);

                this.translationSup = translationSup;
                this.strafeSup = strafeSup;
                this.rotationXSup = rotationSupX;
                this.rotationYSup = rotationSupY;

                this.toggleFieldOriented = toggleFieldOriented;
        }
        double lastAngle = 0.0;
        @Override
        public void execute() {
                if (toggleFieldOriented.getAsBoolean()) {
                        isFieldOriented = !isFieldOriented;
                }

                double translationVal = translationLimiter.calculate(
                                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
                double strafeVal = strafeLimiter.calculate(
                                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
                // double rotationVal = rotationLimiter.calculate(
                                // MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
                
                // Calculate the desired angle based on joystick X (strafe) and Y (translation)
                // inputs
                // Using atan2 to get the angle of the vector (strafe, translation) in radians
                double desiredAngle = Math.atan2(strafeVal, translationVal); // atan2(y, x) for angle calculation
                if(desiredAngle == 0.0) {
                        desiredAngle = lastAngle;
                } else {
                        lastAngle = desiredAngle;
                }
                // Calculate the translation speed as the distance from (0, 0) to
                // (translationVal, strafeVal)
                double translationSpeed = Math.sqrt(Math.pow(translationVal, 2) + Math.pow(strafeVal, 2));

                // Optionally, you can scale translationSpeed by maxSpeed if needed
                translationSpeed = translationSpeed * Constants.Swerve.maxSpeed;

                // If rotationVal is not zero, we want to apply rotation logic
                // if (rotationVal != 0.0) {
                //         double[] wheelAngles = { Math.PI / 4, 3 * Math.PI / 4, -Math.PI / 4, -3 * Math.PI / 4 }; // 45°,
                //         double[] wheelSpeeds = new double[4];

                //         for (int i = 0; i < 4; i++) {
                //                 double wheelSpeed = Math.sqrt(Math.pow(translationSpeed, 2) + Math.pow(rotationVal, 2));
                //                 wheelSpeeds[i] = wheelSpeed;
                //                 s_Swerve.setWheelAngle(i, wheelAngles[i]); // Set the wheel angles
                //         }
                //         for (int i = 0; i < 4; i++) {
                //                 s_Swerve.setWheelSpeed(i, wheelSpeeds[i]); // Set the wheel speed
                //         }
                // } else {
                        // Set the robot's direction and speed using s_Swerve methods
                        double angleDegrees = Math.toDegrees(desiredAngle);
                        
                        s_Swerve.setDirection(angleDegrees); // Pass the desired angle (in radians)
                        s_Swerve.setSpeed(-translationSpeed); // Pass the calculated translation speed

                // }
        }

}