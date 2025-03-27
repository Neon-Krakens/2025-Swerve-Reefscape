package frc.robot.subsystems.Bot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    private final SparkMax stickMotor;
    private final DigitalInput hasCoral = new DigitalInput(0);

    public IntakeArm() {
        stickMotor = new SparkMax(14, MotorType.kBrushless);
        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        stickMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        stickMotor.getEncoder().setPosition(0.0);

        SmartDashboard.putBoolean("Zero Arm", false);
    }

    double speed = 0.0;
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

    public Command setSpeed(double speed) {
        return Commands.run(() -> {
            this.speed = speed;
        });
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
    public boolean elevatorChanged = false;
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("LIMIT HIT", !hasCoral.get());
        if(DriverStation.isDisabled()) return;

        if(!hasCoral.get() && !lastHasCoral) {
            lastHasCoral = true;
            loadingPosition();
            elevatorChanged = false;
        }
        
        if(hasCoral.get() && elevatorChanged) {
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