package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private final SparkMax stickMotor;
    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Algae() {
        stickMotor = new SparkMax(14, MotorType.kBrushless);
        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        stickMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        stickMotor.getEncoder().setPosition(0.0);
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
    @Override
    public void periodic() {
        var position = stickMotor.getEncoder().getPosition();
        var speed = (target-position)/10.0;
        var atTarget = Math.abs(target-position)<0.1;

        if(!atTarget) timeNotAtTarget++;
        else timeNotAtTarget = 0;
        
        if(speed>0.005) {
            speed = 0.005; // Slow when going down
            timeNotAtTarget = 0;
        }
        // starts a 1.01x multiplier after 2 seconds of not reaching goal
        if(timeNotAtTarget>50) { 
            System.out.println("Not at target, multiplying "+(1+((timeNotAtTarget/50.0)/3.0)));
            
            // 50/s, will be at 1.6x after 1s
            speed = speed*(1+((timeNotAtTarget/50)/3));
            System.out.println("Speed: "+speed);
        }

        stickMotor.set(speed);
    }
}