package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Bot.IntakeArm;
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
  private final SendableChooser<String> autos = new SendableChooser<>();
  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  private final Elevator elevator = new Elevator();
  private final Coral coral = new Coral();
  private final Climb climb = new Climb();
  private final IntakeArm algae = new IntakeArm();
  private final LightSubsystem lights = new LightSubsystem();

  /**
   * Input stream for swerve drive control.
   * Configures how controller inputs are processed and applied to drive commands.
   */
  public SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> driver.getLeftY() * 1,
      () -> driver.getLeftX() * 1)
      .withControllerHeadingAxis(() -> driver.getRightX() * 1, () -> driver.getRightY() * 1)
      .deadband(Constants.DRIVER_DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true)
      .headingWhile(true);

  public SwerveInputStream driveInputStreamRobotStupid = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> driver.getLeftY() * 1,
      () -> driver.getLeftX() * 1)
      .scaleTranslation(0.5)
      .withControllerRotationAxis(() -> driver.getRightX() * -1)
      .deadband(Constants.DRIVER_DEADBAND)
      // .allianceRelativeControl(true)
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

    autos.addOption("Left2", "LEFT2");
    autos.setDefaultOption("Middle", "MID");
    autos.addOption("Right2", "RIGHT2");

    // Add the chooser to Shuffleboard
    SmartDashboard.putData("Auto Chooser", autos);

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
    NamedCommands.registerCommand("Set Elevator 4", elevator.goToLevel(4));
    NamedCommands.registerCommand("Set Elevator 3", elevator.goToLevel(3));
    NamedCommands.registerCommand("Set Elevator 2", elevator.goToLevel(2));
    NamedCommands.registerCommand("Set Elevator 1", elevator.goToLevel(1));

    NamedCommands.registerCommand("Wait For Coral", algae.waitForCoral());

  
    NamedCommands.registerCommand("Shoot Coral", coral.spinWheelSequence());
    NamedCommands.registerCommand("Lock Coral", coral.lockInCoral());
    
    NamedCommands.registerCommand("Catcher Out", Commands.runOnce(()->{
      algae.loadingPosition();
    }, algae));

    NamedCommands.registerCommand("Catcher In", Commands.runOnce(()->{
      algae.dropPosition();
    }, algae));
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

    driver.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));

    driver.y().toggleOnTrue(swerveDrive.cancelPathfinding());
    driver.x().toggleOnTrue(swerveDrive.goToClosestCoralTag(true));
    driver.b().toggleOnTrue(swerveDrive.goToClosestCoralTag(false));

    driver.a().toggleOnTrue(
      Commands.runOnce(()->{
        elevator.goDownLevel();
        elevator.goDownLevel();
        elevator.goDownLevel();
        elevator.goDownLevel();

        algae.dropPosition();
        algae.setElevatorChanged();
      }, elevator)
    );

    driver.leftBumper().toggleOnTrue(Commands.runOnce(()->{
        elevator.goDownLevel();

        if(elevator.getLevel() == 1) {
          algae.dropPosition();
          algae.setElevatorChanged();
        } else {
          algae.loadingPosition();
        }
      }, elevator)
    );

    driver.rightBumper().toggleOnTrue(Commands.runOnce(()->{
        elevator.goUpLevel();

        if(elevator.getLevel() == 1) {
          algae.dropPosition();
        } else {
          algae.loadingPosition();
          if(elevator.getLevel() == 2) {
            // coral.lockInCoral();
            
            CommandScheduler.getInstance().schedule(
              Commands.sequence(
                Commands.waitSeconds(0.5),
                coral.spinForward(),
                Commands.waitSeconds(0.3),
                coral.spinStop()
              )
            );
          }
        }
      }, elevator)
    );
    
    driver.rightTrigger()
      .whileTrue(coral.spinForward())
      .onFalse(coral.spinStop());
    driver.leftTrigger()
      .whileTrue(coral.spinReverse())
      .onFalse(coral.spinStop());

    

    // driver.leftTrigger(0.0).whileTrue(
    //   Commands.run(()->{
    //     algae.setPosition(driver.getLeftTriggerAxis());
    //   },algae)
    // );

    driver.povUp().whileTrue(climb.bringInClimber());
    driver.povDown().whileTrue(climb.deployClimberOut());

    driver.povLeft().whileTrue(swerveDrive.scootLeft());
    driver.povRight().whileTrue(swerveDrive.scootRight());
  }

  /**
   * Provides the command to run during autonomous mode.
   * Currently returns a placeholder command that prints a message,
   * indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    String selectedAuto = autos.getSelected();

    switch (selectedAuto) {
      case "LEFT1":
        return new PathPlannerAuto("Left Auto 1 Coral");
      case "LEFT2":
        return new PathPlannerAuto("Left Auto 2 Coral");
      case "MID":
        return new PathPlannerAuto("Mid Auto");
      case "RIGHT1":
        return new PathPlannerAuto("Right Auto 1 Coral");
      case "RIGHT2":
        return new PathPlannerAuto("Right Auto 2 Coral");
      default:
        break;
    }

    return new PathPlannerAuto("Mid Auto");
  }

  public CommandXboxController getController() {
    return driver;
  }
}