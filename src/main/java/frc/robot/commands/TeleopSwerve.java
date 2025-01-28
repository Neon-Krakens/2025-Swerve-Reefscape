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

                double translationVal = translationLimiter.calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
                double strafeVal = strafeLimiter.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));

                // Calculate the desired angle based on joystick X (strafe) and Y (translation)
                double desiredAngle = Math.atan2(strafeVal, translationVal);

                // Calculate the translation speed as the distance from (0, 0) to
                double translationSpeed = Math.sqrt(Math.pow(translationVal, 2) + Math.pow(strafeVal, 2));

                // Optionally, you can scale translationSpeed by maxSpeed if needed
                translationSpeed = translationSpeed * Constants.Swerve.maxSpeed;

                double angleDegrees = Math.toDegrees(desiredAngle);

                s_Swerve.setAllWheelDirection(angleDegrees);
                s_Swerve.setAllWheelSpeed(-translationSpeed);


                double rotationX = rotationLimiter.calculate(MathUtil.applyDeadband(rotationXSup.getAsDouble(), Constants.Swerve.stickDeadband));
                double rotationY = rotationLimiter.calculate(MathUtil.applyDeadband(rotationYSup.getAsDouble(), Constants.Swerve.stickDeadband));

                // Calculate the desired angle based on joystick X (strafe) and Y (translation)
                double desiredBotAngle = Math.atan2(rotationX, rotationY);

                // Calculate the translation speed as the distance from (0, 0) to
                double botRotationSpeed = Math.sqrt(Math.pow(translationVal, 2) + Math.pow(strafeVal, 2));

                // Optionally, you can scale translationSpeed by maxSpeed if needed
                botRotationSpeed = botRotationSpeed * Constants.Swerve.maxSpeed;

                double angleBotDegrees = Math.toDegrees(desiredBotAngle);

                s_Swerve.setBotDirection(angleBotDegrees);
        }

}