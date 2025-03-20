package frc.robot.subsystems.Bot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    private int targetLevel = 1;
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
        // rightConfig.idleMode(IdleMode.kBrake);
        rightLift.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // var leftConfig = new SparkMaxConfig();
        // leftConfig.idleMode(IdleMode.kBrake);
        // leftLift.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SmartDashboard.putBoolean("Reset Elevator", false);
    }
    
    double liftSpeed = 0.0;


    @Override
    public void periodic() {
        double position = leftLift.getEncoder().getPosition();

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

        if (Math.abs(distance) < 0.5) {
            if(target==0) leftLift.set(0.02);
            else leftLift.set(0.04);
            atTarget = true;
            return;
        }
        atTarget = false;

        double speed = Math.max(Math.min(distance / 38.0, 0.3), -0.2);
        if (speed < 0 && speed > -0.05) speed = -0.05; // Min speed when going dowb
        if (speed > 0 && speed < 0.1) speed = 0.1;  // Min speed when going up
        if(speed < 0.2 && targetLevel == 4) speed = 0.2;

        liftSpeed = -speed;
        System.out.println("SETTING LIFT SPEED: "+speed);
        leftLift.set(speed);
    }

    // @Override
    // public void simulationPeriodic() {
    //     if (Robot.isSimulation()) {
    //         double last = leftLift.getEncoder().getPosition(); // 0 to 660\
    //         if(Math.abs(target - last) < 10) {
    //             atTarget = true;
    //             return;
    //         }
    //         leftLift.getEncoder().setPosition(last+liftSpeed*30);
    //     }
    // }

    private Command setTargetLevel(int level) {
        return new FunctionalCommand(
            () -> {
                targetLevel = Math.max(1, Math.min(level, 4)); // Keep within bounds
                if(targetLevel == 1) target = 0; // Not raised and bottom corla
                // if(targetLevel == 2) target = 52;//2.3
                // if(targetLevel == 3) target = 295;
                // if(targetLevel == 4) target = 600;
                
                if(targetLevel == 2) target = 2.3;//2.3
                if(targetLevel == 3) target = 16.5;//16.5
                if(targetLevel == 4) target = 37.5;//38.5
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
    public int getLevel() {
        return targetLevel;
    }
}