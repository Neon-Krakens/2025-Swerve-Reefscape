package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.Vision;

public class RobotContainer {
    private final Vision vision = new Vision();
    public final Swerve s_Swerve = new Swerve(vision);
    private final XboxController driverController = new XboxController(0);

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(
        s_Swerve,
        () -> -driverController.getLeftY(), // forward/backward
        () -> driverController.getLeftX(), // left/right
        () -> driverController.getRightX(), // rotation
        driverController::getStartButton // Toggle field-oriented mode with Start button
    );

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Create a custom Trigger for Start + A button press
        Trigger startAndAButton = new Trigger(() ->
            driverController.getStartButton() && driverController.getAButton()
        );

        startAndAButton.onTrue(new InstantCommand(s_Swerve::zeroGyro, s_Swerve));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getTeleopCommand() {
        return teleopSwerve;
    }
}