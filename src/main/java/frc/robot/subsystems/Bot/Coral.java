package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
    private final SparkMax wheelMotor;
    // private final DigitalInput hasCoral = new DigitalInput(0);

    public Coral() {
        wheelMotor = new SparkMax(11, MotorType.kBrushless);
    }

    public Command spinWheelSequence() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                wheelMotor.set(-0.35);
                System.out.println("Coral Started");
            }), // Start spinning
            Commands.waitSeconds(0.75),               // Wait for 3 seconds
            Commands.runOnce(() -> {
                wheelMotor.set(0);
                System.out.println("Coral stopped");
            })// Stop after 3 seconds
        );
    }
    public Command lockInCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                wheelMotor.set(-0.35);
                System.out.println("Coral Started");
            }), // Start spinning
            Commands.waitSeconds(0.25),               // Wait for 3 seconds
            Commands.runOnce(() -> {
                wheelMotor.set(0);
                System.out.println("Coral stopped");
            })// Stop after 3 seconds
        );
    }

    public Command spinForward() {
        return Commands.runOnce(() -> {
            double wheelSpeed = 0.3;
            wheelMotor.set(-wheelSpeed);
            System.out.println("Coral Forward Started at speed: " + wheelSpeed);
        });
    }

    public Command spinStop() {
        return Commands.runOnce(() -> {
            wheelMotor.set(0.0);
            System.out.println("Coral Stopped");
        });
    }

    @Override
    public void periodic() {

        // wheelMotor.set(0.0);
    }

    public Command spinReverse() {
        return Commands.runOnce(() -> {
            double wheelSpeed = -0.3;
            wheelMotor.set(wheelSpeed);
            System.out.println("Coral Reverse Started at speed: " + wheelSpeed);
        });
    }
}