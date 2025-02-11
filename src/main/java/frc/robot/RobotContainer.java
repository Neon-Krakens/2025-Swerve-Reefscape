package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain.Swerve;
import swervelib.SwerveInputStream;

/**
 * Main robot configuration class that binds controls and commands to subsystems.
 * This class serves as the robot's command center, managing all subsystem instances
 * and their associated commands.
 * 
 * <p>Features include:
 * <ul>
 *   <li>Driver control configuration
 *   <li>Command button mappings
 *   <li>Autonomous command selection
 *   <li>Subsystem instantiation and management
 * </ul>
 * 
 * <p>The class follows a centralized control pattern, with all robot behaviors
 * defined through command bindings and default commands.
 */
public class RobotContainer {
    /** Xbox controller used for driver input. */
    private final CommandXboxController driver = new CommandXboxController(0);
  
    /** Main drive subsystem for robot movement. */
    private final Swerve swerveDrive = Swerve.getInstance();
  
    /** 
     * Input stream for swerve drive control.
     * Configures how controller inputs are processed and applied to drive commands.
     */
    private final SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
        () -> driver.getLeftY() * -1,
        () -> driver.getLeftX() * -1)
        .cubeTranslationControllerAxis(true)
        .scaleTranslation(0.5)
        .cubeRotationControllerAxis(true)
        .withControllerHeadingAxis(() -> driver.getRightX() * -1, () -> driver.getRightY() * -1)
        .cubeRotationControllerAxis(true)
        .deadband(Constants.DRIVER_DEADBAND)
        .allianceRelativeControl(true)
        .headingWhile(true);
  
    /**
     * Creates a new RobotContainer and initializes all robot subsystems and commands.
     * Performs the following setup:
     * <ul>
     *   <li>Silences joystick warnings for unplugged controllers
     *   <li>Disables controller rumble
     *   <li>Configures button bindings for commands
     * </ul>
     */
    public RobotContainer() {
      DriverStation.silenceJoystickConnectionWarning(true);
      driver.setRumble(RumbleType.kBothRumble, 0.0);
  
      configureBindings();
    }
  
    /**
     * Configures button bindings for commands.
     * Maps controller buttons to specific robot actions:
     * <ul>
     *   <li>Default command: Field oriented drive using joystick input
     *   <li>X button: Lock wheels in X pattern for stability
     *   <li>Start button: Reset odometry to field center
     *   <li>Back button: Autonomous drive to field center
     * </ul>
     */
    private void configureBindings() {
      Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveInputStream);
      swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);
  
      driver.x().whileTrue(Commands.runOnce(swerveDrive::lockWheels, swerveDrive).repeatedly());
      driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));
      driver.back().whileTrue(
          swerveDrive.driveToPose(
              new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),
                  Rotation2d.fromDegrees(0))));
    }
  
    /**
     * Provides the command to run during autonomous mode.
     * Currently returns a placeholder command that prints a message,
     * indicating no autonomous routine is configured.
     *
     * @return the command to run in autonomous mode
     */
    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }
  }