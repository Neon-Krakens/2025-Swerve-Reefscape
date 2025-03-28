package frc.robot.subsystems.Bot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeArm extends SubsystemBase {
    private final SparkMax stickMotor;
    private final DigitalInput hasCoral = new DigitalInput(0);
    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault().getStructTopic("3d IntakeArm", Pose3d.struct).publish();

    public IntakeArm() {
        stickMotor = new SparkMax(14, MotorType.kBrushless);
        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        stickMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        stickMotor.getEncoder().setPosition(0.0);

        SmartDashboard.putBoolean("Zero Arm", false);
        SmartDashboard.putBoolean("Trigger Limit Switch", false);
    }

    double target = 0.0;

    public Command waitForCoral() {
        return new FunctionalCommand(
            () -> {},
            () -> {},
            interrupted -> {},
            () -> !hasCoral.get(), // Ends when at the target
            this
        );
    }

    public Command setPosition(double percent) {
        target = (percent*(-2.1));
        return Commands.none();
    }

    int timeNotAtTarget = 0; // 50/s, 
    boolean roundStarted = false;
    double loadingPosition = 8.009;
    double dropPosition = 6.7285;

    public void loadingPosition() {
        target = loadingPosition;
        System.out.println("SETTING TO LOADING POSITION");
    }

    public void dropPosition() {
        target = dropPosition;
        System.out.println("SETTING TO DROP POSITION");
    }
    
    double lastPosition = 0.0;
    long zeroedAt = 0L;
    boolean lastHasCoral = false;

    public void setElevatorChanged() {
        elevatorChanged = true;
    }
    boolean elevatorChanged = false;

    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            double last = stickMotor.getEncoder().getPosition(); // -5

            stickMotor.getEncoder().setPosition((target-last)/10);

            if(target == loadingPosition) {
                var pose3d = Swerve.getInstance().getPose3d().transformBy(new Transform3d(0, 0.3, Elevator.pose3d.getZ(), new Rotation3d(0,Math.toRadians(90),Math.toRadians(90))));
                publisher.set(pose3d);
            } else if(target == dropPosition) {
                var pose3d = Swerve.getInstance().getPose3d().transformBy(new Transform3d(0, 0.3, Elevator.pose3d.getZ(), new Rotation3d(0,Math.toRadians(130),Math.toRadians(90))));
                publisher.set(pose3d);
            } else {
                var pose3d = Swerve.getInstance().getPose3d().transformBy(new Transform3d(0, 0.3, Elevator.pose3d.getZ(), new Rotation3d(0,Math.toRadians(-90),Math.toRadians(90))));
                publisher.set(pose3d);
            }
        }
    }

    boolean isLimitSwitchHit() {
        if(Robot.isSimulation()) {
            return SmartDashboard.getBoolean("Trigger Limit Switch", false);
        }
        return !hasCoral.get();
    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LIMIT HIT", isLimitSwitchHit());
        if(DriverStation.isDisabled()) return;

        if(isLimitSwitchHit() && !lastHasCoral) {
            lastHasCoral = true;
            loadingPosition();
            elevatorChanged = false;
        }
        
        if(!isLimitSwitchHit() && elevatorChanged) {
            lastHasCoral = false;
            dropPosition();
        }

        elevatorChanged = false;

        if (SmartDashboard.getBoolean("Zero Arm", false)) {
            if(zeroedAt == 0L) zeroedAt = System.currentTimeMillis();

            if(System.currentTimeMillis() - zeroedAt > 2_000) {
                stickMotor.set(0.0);
                if(System.currentTimeMillis() - zeroedAt > 2_500) {
                    SmartDashboard.putBoolean("Zero Arm", false);
                    zeroedAt = 0L;
                    System.out.println("ZEROD ARM");
                    stickMotor.getEncoder().setPosition(0.0);
                }
                return;
            } else {
                System.out.println("ZEROING ARM");
                stickMotor.set(0.04);
                return;
            }
        }
        
        var position = -stickMotor.getEncoder().getPosition();

        SmartDashboard.putNumber("Arm Position", position);

        if(DriverStation.isEnabled() && !roundStarted) {
            roundStarted = true;
            dropPosition();
        }

        var dist = target-position;

        if(Math.abs(dist) <= 0.2) {
            // At the target
            stickMotor.set(-0.016); // was -0.013
            return;
        }
        
        if(dist > 0) {
            // Upward
            stickMotor.set(-0.08);
        } else {
            stickMotor.set(0.05);
        }
        
    }
}