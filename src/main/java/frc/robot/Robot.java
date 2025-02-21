package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Main robot class that manages the robot's lifecycle and operational modes.
 * This class follows the TimedRobot model, executing code in a timed loop for
 * each robot mode.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Autonomous command management
 * <li>Teleop control initialization
 * <li>Test mode handling
 * <li>Periodic updates across all modes
 * </ul>
 * 
 * <p>
 * The robot code is organized using WPILib's command-based framework, with
 * subsystems
 * and commands managed through the RobotContainer class.
 */
public class Robot extends TimedRobot {
  /** The autonomous command to run. */
  private Command autonomousCommand;

  /** The robot's container, which holds subsystems and commands. */
  private final RobotContainer robotContainer;

  /**
   * Creates a new Robot and initializes the RobotContainer.
   */
  public Robot() {
    robotContainer = new RobotContainer();
  }

  /**
   * Periodic function called every 20ms in all robot modes.
   * Runs the command scheduler to execute ongoing commands and update subsystems.
   * This method is crucial for continuous robot operation and should not be
   * blocked
   * or delayed.
   */
  boolean lastColor = false;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    int estimatedPercentage = (int) ((RobotController.getBatteryVoltage() - 11.3) / (12.8 - 11.3) * 100.0);
    SmartDashboard.putString("Est. Battery", estimatedPercentage + "%");

    double timeRemaining = Math.max(DriverStation.getMatchTime() - 30, 0);
    SmartDashboard.putNumber("Time Till Climb", timeRemaining);
    SmartDashboard.putBoolean("GO CLIMB", lastColor);

    if (timeRemaining == 0) {
      lastColor = ((int) (DriverStation.getMatchTime() / 0.5)) % 2 == 0;
    } else {
      lastColor = false;
    }

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** Called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** Called periodically when the robot is disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** Called once when disabled mode is exited. */
  @Override
  public void disabledExit() {
  }

  /**
   * Called when autonomous mode is initialized.
   * Retrieves and schedules the selected autonomous command from RobotContainer.
   * If no command is selected, this method will gracefully handle the null case.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** Called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {
  }

  /** Called once when autonomous mode is exited. */
  @Override
  public void autonomousExit() {
  }

  /**
   * Called when operator control is initialized.
   * Cancels any running autonomous command to ensure safe transition to manual
   * control.
   * This prevents autonomous commands from interfering with driver inputs.
   */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** Called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** Called once when operator control is exited. */
  @Override
  public void teleopExit() {
  }

  /**
   * Called when test mode is initialized.
   * Cancels all running commands to ensure a clean slate for testing.
   * This helps prevent interference from commands that might have been
   * left running in other modes.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** Called once when test mode is exited. */
  @Override
  public void testExit() {
  }
}