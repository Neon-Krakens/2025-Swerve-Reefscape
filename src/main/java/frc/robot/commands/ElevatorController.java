package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorController extends SubsystemBase {
    private double target = 0.0;
    private int targetLevel = 0;
    private boolean atTarget = false;
    private final SparkMax rightLift;
    private final SparkMax leftLift;
    private final DigitalInput LimitSwitchBottom = new DigitalInput(Constants.ELEVATOR_LIMITSWITCH_CHANNEL_TOP);
    private final DigitalInput LimitSwitchTop = new DigitalInput(Constants.ELEVATOR_LIMITSWITCH_CHANNEL_BOTTOM);
    
    private final SparkMaxConfig rightConfig;

    public ElevatorController() {
        rightLift = new SparkMax(9, MotorType.kBrushless);
        leftLift = new SparkMax(10, MotorType.kBrushless);

        // Setup configuration for the left motor

        // leftLift.setInverted(true);

        rightConfig = new SparkMaxConfig();
        rightConfig.follow(leftLift, true);
        rightLift.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //leftLift.getEncoder().setPosition(0.0); // Only zero elevator after work is done on it

        SmartDashboard.putBoolean("Reset Elevator", false);
    }

    @Override
    public void periodic() {
        double position = -leftLift.getEncoder().getPosition();

        SmartDashboard.putNumber("Elevator Target Level",targetLevel);
        SmartDashboard.putNumber("Elevator Pos",position);
        SmartDashboard.putNumber("Elevator Target",target);

        if(SmartDashboard.getBoolean("Reset Elevator", false)) {
            System.out.println("RESET ELEVATOR");
            leftLift.getEncoder().setPosition(0.0);
            SmartDashboard.putBoolean("Reset Elevator", false);
        }

        // VERY TOP: 755, new 660
        // SAFE TOP: 745, new 660
        // VERY BOTTOM: 0;

        // Stop if position is close
        double distance = target - position;

        if (Math.abs(distance) < 5) {// || (LimitSwitchBottom.get() || LimitSwitchTop.get())) {
            leftLift.set(0.0);
            atTarget = true;
            return;
        }

        double speed = distance / 630.0; // 
        speed = -speed; // Invert motor, and make slowest speed 0.1
        if(speed < 0 && speed > -0.1) speed = -0.1;
        if(speed > 0 && speed < 0.2) speed = 0.2;

        System.out.println("ELEVATOR SPEED:"+speed);
        leftLift.set(speed);
    }

    private void goToLevel(int level) {
        target = level * (660.0 / 4);
    }

    public void goUpLevel() {
        System.out.println("GO UP LEVEL");
        CommandScheduler.getInstance().schedule(
            new FunctionalCommand(
                // Reset encoders on command start
                () -> {
                    if (targetLevel == 4) return;
                    System.out.println("GOING TO LEVEL: " + targetLevel);

                    targetLevel++;
                    goToLevel(targetLevel);
                    atTarget = false;
                },
                // Start driving forward at the start of the command (or do something else here)
                () -> {

                },
                // Stop driving at the end of the command (or do something else here)
                success -> {
                    System.out.println("IM AT LEVEL: " + targetLevel);
                },
                // End the command when the robot's driven distance exceeds the desired value
                () -> atTarget,
                // No subsystem needed, so pass null or omit this parameter
                this)
        );
    }

    public void goDownLevel() {
        System.out.println("GO DOWN LEVEL");
        CommandScheduler.getInstance().schedule(new FunctionalCommand(
                // Reset encoders on command start
                () -> {
                    if (targetLevel == 0)
                        return;
                    System.out.println("GOING TO LEVEL: " + targetLevel);

                    targetLevel--;
                    goToLevel(targetLevel);
                    atTarget = false;
                },
                // Start driving forward at the start of the command (or do something else here)
                () -> {

                },
                // Stop driving at the end of the command (or do something else here)
                success -> {
                    System.out.println("IM AT LEVEL: " + targetLevel);
                },
                // End the command when the robot's driven distance exceeds the desired value
                () -> atTarget,
                // No subsystem needed, so pass null or omit this parameter
                this)
            );
    }
}