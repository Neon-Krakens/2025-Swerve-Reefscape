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
    private final SparkMax climbMotorLeft;
    private final SparkMax climbMotorRight;

    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Climb() {
        climbMotorLeft = new SparkMax(15, MotorType.kBrushless);
        climbMotorRight = new SparkMax(16, MotorType.kBrushless);

        // Setup configuration for the left motor

        // leftLift.setInverted(true);

        var rightConfig = new SparkMaxConfig();
        rightConfig.follow(climbMotorRight, true);
        rightConfig.idleMode(IdleMode.kBrake);
        climbMotorLeft.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // var leftConfig = new SparkMaxConfig();
        // leftConfig.idleMode(IdleMode.kBrake);
        // climbMotorRight.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    Long lastPressedIn = 0L;
    public Command bringInClimber() {
        // Reset the scaling if last presssed over 5 seconds ago
        if(System.currentTimeMillis()-lastPressedIn > 5_000) {
            speed = -0.5;
        }

        lastPressedIn = System.currentTimeMillis();
        return Commands.run(() -> {
            this.speed -= 0.075; 
            System.out.println("Climb going in at speed: " + this.speed);
        });
    }

    public Command deployClimberOut() {
        return Commands.run(() -> {
            this.speed = 1; 
            System.out.println("Climb going out at speed: " + this.speed);
        });
    }

    double speed = 0.0;
    // public Command setVoltage(double speed) {
    //     return Commands.run(() -> {
    //         this.speed = speed; 
    //     });
    // }

    @Override
    public void periodic() {
        climbMotorRight.setVoltage(speed);
        speed = 0.0;
    }
}