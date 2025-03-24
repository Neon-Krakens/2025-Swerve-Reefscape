package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final SparkMax climbMotor;

    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Climb() {
        climbMotor = new SparkMax(15, MotorType.kBrushless);

        var rightConfig = new SparkMaxConfig();
    
        rightConfig.idleMode(IdleMode.kBrake);
        climbMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    Long lastPressedIn = 0L;
    double speed2 = 0.0;
    public Command bringInClimber() {
        return Commands.run(() -> {
            this.speed = 0.75; // going up
            System.out.println("Climb going up at speed: " + this.speed);
        });
    }

    public Command deployClimberOut() {
        return Commands.run(() -> {
            this.speed = -1;
            System.out.println("Climb going down at speed: " + this.speed);
        });
    }

    double speed = 0.0;
    
    @Override
    public void periodic() {
        climbMotor.set(speed);
        speed = 0.0;
    }
}