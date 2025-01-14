package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private final Swerve s_Swerve;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier toggleFieldOriented;

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private boolean isFieldOriented = true; // Field oriented means the robot drives relative to the field, not the robot

    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier toggleFieldOriented) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.toggleFieldOriented = toggleFieldOriented;

        // Add field-oriented mode to Shuffleboard
        Shuffleboard.getTab("Field").addBoolean("Field Oriented", () -> isFieldOriented);
    }

    @Override
    public void execute() {
        if (toggleFieldOriented.getAsBoolean()) {
            isFieldOriented = !isFieldOriented;
        }

        double translationVal = translationLimiter.calculate(
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
        double strafeVal = strafeLimiter.calculate(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
        double rotationVal = rotationLimiter.calculate(
                MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                isFieldOriented,
                true);
    }
}