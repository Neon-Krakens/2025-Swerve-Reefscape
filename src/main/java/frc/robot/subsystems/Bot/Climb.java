package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private final SparkMax wheelMotor;
    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Climb() {
        wheelMotor = new SparkMax(13, MotorType.kBrushless);
    }

    double speed = 0.0;
    public Command setSpeed(double speed) {
        return Commands.run(() -> {
            this.speed = speed;
            System.out.println("Climb Started at speed: " + speed);
        });
    }

    @Override
    public void periodic() {
        wheelMotor.set(speed);
        speed = 0.0;
    }
}