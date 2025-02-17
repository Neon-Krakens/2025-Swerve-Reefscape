package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private final SparkMax wheelMotor;
    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Coral() {
        wheelMotor = new SparkMax(11, MotorType.kBrushless);
    }

    public Command spinWheel() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                wheelMotor.set(0.5);
                System.out.println("Coral Started");
            }), // Start spinning
            Commands.waitSeconds(3),               // Wait for 3 seconds
            Commands.runOnce(() -> {
                wheelMotor.set(0);
                System.out.println("Coral stopped");
            })// Stop after 3 seconds
        );
    }
}