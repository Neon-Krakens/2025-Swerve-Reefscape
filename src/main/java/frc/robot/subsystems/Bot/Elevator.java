package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
    private double target = 0.0;
    private int targetLevel = 0;
    private boolean atTarget = false;
    private final SparkMax rightLift;
    private final SparkMax leftLift;

    // private final DigitalInput limitSwitchBottom = new DigitalInput(Constants.ELEVATOR_LIMITSWITCH_CHANNEL_BOTTOM);
    // private final DigitalInput limitSwitchTop = new DigitalInput(Constants.ELEVATOR_LIMITSWITCH_CHANNEL_TOP);

    private final SparkMaxConfig rightConfig;

    public Elevator() {
        rightLift = new SparkMax(9, MotorType.kBrushless);
        leftLift = new SparkMax(10, MotorType.kBrushless);

        // Setup configuration for the left motor

        // leftLift.setInverted(true);

        rightConfig = new SparkMaxConfig();
        rightConfig.follow(leftLift, true);
        rightLift.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putBoolean("Reset Elevator", false);
    }
    
    double liftSpeed = 0.0;


    @Override
    public void periodic() {
        double position = -leftLift.getEncoder().getPosition();

        SmartDashboard.putNumber("Elevator Target Level", targetLevel);
        SmartDashboard.putNumber("Elevator Pos", position);
        SmartDashboard.putNumber("Elevator Target", target);

        if (SmartDashboard.getBoolean("Reset Elevator", false)) {
            System.out.println("RESET ELEVATOR");
            leftLift.getEncoder().setPosition(0.0);
            SmartDashboard.putBoolean("Reset Elevator", false);
        }

        if(DriverStation.isDisabled()) {
            target = position;
            atTarget = true;
            return;
        }

        // Stop movement when near target
        double distance = target - position;

        if (Math.abs(distance) < 5) {
            leftLift.set(0.0);
            atTarget = true;
            return;
        }
        atTarget = false;

        double speed = Math.max(Math.min(distance / 500.0, 1.0), -1.0);
        if (speed < 0 && speed > -0.17) speed = -0.17; // Min speed when going dowb
        if (speed > 0 && speed < 0.1) speed = 0.1;  // Min speed when going up

        liftSpeed = -speed;
        System.out.println("SETTING LIFT SPEED: "+liftSpeed);
        leftLift.set(liftSpeed);
    }

    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            double last = leftLift.getEncoder().getPosition(); // 0 to 660\
            if(Math.abs(target - last) < 10) {
                atTarget = true;
                return;
            }
            leftLift.getEncoder().setPosition(last+liftSpeed*30);
        }
    }

    private Command setTargetLevel(int level) {
        return new FunctionalCommand(
            () -> {
                targetLevel = Math.max(0, Math.min(level, 4)); // Keep within bounds
                target = (targetLevel * (640.0 / 4.0))+10; // make the top most 650, and bottom most 10
                atTarget = false;
                System.out.println("Setting target level: " + targetLevel);
            },
            () -> {
                
            },
            interrupted -> {
                leftLift.set(0.0); // Stop the motor when the command ends
                System.out.println("Reached target level: " + targetLevel);
            },
            () -> atTarget, // Ends when at the target
            this
        );
    }

    public Command goToLevel(int level) {
        return setTargetLevel(level);
    }

    public void goUpLevel() {
        CommandScheduler.getInstance().schedule(goToLevel(targetLevel + 1));
    }

    public void goDownLevel() {
        CommandScheduler.getInstance().schedule(goToLevel(targetLevel - 1));
    }
}