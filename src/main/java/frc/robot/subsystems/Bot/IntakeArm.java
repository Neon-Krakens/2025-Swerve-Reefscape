package frc.robot.subsystems.Bot;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArm extends SubsystemBase {
    private final SparkMax stickMotor;
    // private final DigitalInput hasCoral = new DigitalInput(0);

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
    double dropPosition = 6.4285;

    public void loadingPosition() {
        target = loadingPosition;
    }

    public void dropPosition() {
        target = dropPosition;
    }
    double lastPosition = 0.0;
    long zeroedAt = 0L;
    @Override
    public void periodic() {
        if(DriverStation.isDisabled()) return;

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
            stickMotor.set(-0.013);
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