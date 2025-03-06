package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Bot.Climb;
import frc.robot.subsystems.Bot.Coral;
import frc.robot.subsystems.Bot.Elevator;
import frc.robot.subsystems.Bot.Swerve;
import frc.robot.subsystems.Lighting.LightSubsystem;
import swervelib.SwerveInputStream;

/**
 * Main robot configuration class that binds controls and commands to
 * subsystems.
 * This class serves as the robot's command center, managing all subsystem
 * instances
 * and their associated commands.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Driver control configuration
 * <li>Command button mappings
 * <li>Autonomous command selection
 * <li>Subsystem instantiation and management
 * </ul>
 * 
 * <p>
 * The class follows a centralized control pattern, with all robot behaviors
 * defined through command bindings and default commands.
 */
public class RobotContainer {

  /** Xbox controller used for driver input. */
  private final CommandXboxController driver = new CommandXboxController(0);

  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  private final Elevator elevator = new Elevator();
  private final Coral coral = new Coral();
  private final Climb climb = new Climb();
  private final LightSubsystem lights = new LightSubsystem();

  /**
   * Input stream for swerve drive control.
   * Configures how controller inputs are processed and applied to drive commands.
   */
  public SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .cubeTranslationControllerAxis(true)
      .scaleTranslation(0.5)
      .cubeRotationControllerAxis(true)
      .withControllerHeadingAxis(() -> driver.getRightX() * -1, () -> driver.getRightY() * -1)
      .deadband(Constants.DRIVER_DEADBAND)
      .allianceRelativeControl(true)
      .headingWhile(true);

  /**
   * Creates a new RobotContainer and initializes all robot subsystems and
   * commands.
   * Performs the following setup:
   * <ul>
   * <li>Silences joystick warnings for unplugged controllers
   * <li>Disables controller rumble
   * <li>Configures button bindings for commands
   * </ul>
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    driver.setRumble(RumbleType.kBothRumble, 0.0);

    configureBindings();
    setupPathPlannerCommands();

    CommandScheduler.getInstance().onCommandInitialize(command -> {
      System.out.println("Command Started: " + command.getName());
    });

    CommandScheduler.getInstance().onCommandInterrupt(command -> {
      System.out.println("Command Interrupted: " + command.getName());
    });

    CommandScheduler.getInstance().onCommandFinish(command -> {
      System.out.println("Command Finished: " + command.getName());
    });
  }

  private void setupPathPlannerCommands() {
    // Commands for PathPlanner auto routines
    NamedCommands.registerCommand("Raise Elevator 4", elevator.goToLevel(4));
    NamedCommands.registerCommand("Raise Elevator 3", elevator.goToLevel(3));
    NamedCommands.registerCommand("Raise Elevator 2", elevator.goToLevel(2));
    NamedCommands.registerCommand("Raise Elevator 1", elevator.goToLevel(1));
    NamedCommands.registerCommand("Raise Elevator 0", elevator.goToLevel(0));

    NamedCommands.registerCommand("Deposit Coral", coral.spinWheelSequence());
  }

  /**
   * Configures button bindings for commands.
   * Maps controller buttons to specific robot actions:
   * <ul>
   * <li>Default command: Field oriented drive using joystick input
   * <li>X button: Lock wheels in X pattern for stability
   * <li>Start button: Reset odometry to field center
   * <li>Back button: Autonomous drive to field center
   * </ul>
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveInputStream);
    swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

    driver.povUp().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));

    driver.y().toggleOnTrue(swerveDrive.cancelPathfinding());
    driver.x().toggleOnTrue(swerveDrive.goToClosestCoralTag(true));
    driver.b().toggleOnTrue(swerveDrive.goToClosestCoralTag(false));

    driver.a().toggleOnTrue(
      Commands.sequence(Commands.runOnce(() -> elevator.goToLevel(1), elevator), swerveDrive.goToClosestDrop())
    );

    driver.leftBumper().toggleOnTrue(Commands.runOnce(elevator::goDownLevel, elevator));
    driver.rightBumper().toggleOnTrue(Commands.runOnce(elevator::goUpLevel, elevator));

    driver.rightTrigger().toggleOnTrue(coral.spinWheelSequence());

    driver.povLeft().whileTrue(climb.setSpeed(0.2));
    driver.povRight().whileTrue(climb.setSpeed(-0.2));


    // driver.leftTrigger().whileTrue(coral.spinReverse());
  }

  /**
   * Provides the command to run during autonomous mode.
   * Currently returns a placeholder command that prints a message,
   * indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Start 1, I, 4");
  }

  public CommandXboxController getController() {
    return driver;
  }
}