package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
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
        private final BooleanSupplier resetGyro;

        public final XboxController driverController;
        private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);

        public TeleopSwerve(
                        Swerve s_Swerve,
                        DoubleSupplier translationSup,
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSupX,
                        DoubleSupplier rotationSupY,
                        BooleanSupplier resetGyro,
                        XboxController controller) {
                this.s_Swerve = s_Swerve;
                addRequirements(s_Swerve);

                this.translationSup = translationSup;
                this.strafeSup = strafeSup;
                this.rotationXSup = rotationSupX;
                this.rotationYSup = rotationSupY;
                this.resetGyro = resetGyro;

                this.driverController = controller;
        }

        boolean lastMovementMode = false;

        @Override
        public void execute() {
                if (resetGyro.getAsBoolean()) {
                        s_Swerve.zeroWheels();
                        s_Swerve.zeroGyro();
                }
                boolean movingMovement = Math.abs(translationSup.getAsDouble()) > 0.1
                                || Math.abs(strafeSup.getAsDouble()) > 0.1;

                double rotationX = rotationXSup.getAsDouble();
                double rotationY = rotationYSup.getAsDouble();
                // Calculate the desired angle based on joystick X (strafe) and Y (translation)

                double desiredBotAngle = Math.atan2(rotationY, rotationX);

                // Add 90 degrees (convert to radians for math operations)
                double angleBotDegrees = Math.toDegrees(desiredBotAngle) + 90;

                // Normalize the result to be within the bounds of [-180, 180]
                if (angleBotDegrees > 180) {
                        angleBotDegrees -= 360; // If it's greater than 180, subtract 360 to bring it into the
                                                // range
                } else if (angleBotDegrees < -180) {
                        angleBotDegrees += 360; // If it's less than -180, add 360 to bring it into the range
                }

                if (Math.abs(rotationX) + Math.abs(rotationY) > 0.3) {
                        s_Swerve.setBotDirection(angleBotDegrees);
                }

                double translationVal = translationLimiter.calculate(MathUtil
                                .applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
                double strafeVal = strafeLimiter.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(),
                                Constants.Swerve.stickDeadband));


                if (Math.abs(translationVal) + Math.abs(strafeVal) > 0.3) {
                        s_Swerve.rotateDrive(strafeVal, translationVal);
                }
                
                s_Swerve.update();
        }

}