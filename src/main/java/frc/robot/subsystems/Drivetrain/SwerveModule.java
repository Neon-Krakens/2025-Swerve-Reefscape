package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.SwerveModuleConfig.ModuleConfig;

/*
 * Modified from https://github.com/DIEHDZ/Swerve-base-Crescendo-2024/blob/master/src/main/java/frc/robot/subsystems/Drivetrain/SwerveModule.java
 */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax rotationMotor; // Rotation Neo Motor Controller
    private SparkMax driveMotor; // Drive Neo Motorf Controller

    private RelativeEncoder rotationEncoder; // Encoder from the NEO motor
    private RelativeEncoder driveEncoder; // Encoder from the NEO motor
    
    private CANcoder absoluteEncoder; // CANcoder located above the swerve module

    public SwerveModule(int moduleNumber, ModuleConfig moduleConfig) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConfig.angleOffset;

        // CANCoder config
        this.absoluteEncoder = new CANcoder(moduleConfig.canCoderID);
        Swerve.configureCANcoder(absoluteEncoder, moduleConfig.canCoderInvert);

        // rotation motor config
        this.rotationMotor = new SparkMax(moduleConfig.rotationMotorID, MotorType.kBrushless);
        this.rotationEncoder = rotationMotor.getEncoder();
        configRotationMotor(moduleConfig.rotationInvert);

        // drive Motor Config
        this.driveMotor = new SparkMax(moduleConfig.driveMotorID, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        configDriveMotor(moduleConfig.driveInvert);

    }

    public double wheelAngle = 0.0; 
    public boolean newOffset = false;
    
    public void updateWheelAngle() {
        if(!newOffset) {
            System.out.println("MOD "+moduleNumber+
            ": OLD:"+angleOffset.getDegrees()+
            " NEW:"+getCanCoder().getDegrees());
            angleOffset = Rotation2d.fromDegrees(getCanCoder().getDegrees());
            newOffset = true;
        }
        wheelAngle = (rotationEncoder.getPosition()*360 - angleOffset.getDegrees()) %360;
        if(moduleNumber == 1) {
            System.out.println("MOD "+moduleNumber+": ANGLE:"+wheelAngle+" "+angleOffset.getDegrees());
        }
    }

    public boolean setAngle(double targetAngle, boolean driveMode) {
        targetAngle = targetAngle + 180; // convery from -180,180 to 0,360
        double currentAngle = wheelAngle - angleOffset.getDegrees();  // Adjust current angle by the offset
        currentAngle = ((currentAngle % 360 + 360) % 360);
        targetAngle = ((targetAngle) % 360 + 360) % 360;  // Adjust target angle by the offset
        double deltaAngle = ((targetAngle - currentAngle + 540) % 360) - 180;
        double speed = deltaAngle / 180.0;  // Scales the speed to [-1, 1] as deltaAngle ranges from -180 to 180
        speed = Math.max(-Constants.Swerve.maxWheelRotateSpeed, Math.min(Constants.Swerve.maxWheelRotateSpeed, speed));
        setRotationSpeed(speed);
        if(driveMode) {
            if(speed > 0.03) {
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    public void setRotationSpeed(double speed) {
        rotationMotor.set(speed);
    }

    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees();
        rotationEncoder.setPosition(absolutePosition);
    }

    private void configRotationMotor(boolean rotationInvert) {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(rotationInvert)
            .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
            .idleMode(Constants.Swerve.angleNeutralMode);

        config.encoder
            .positionConversionFactor(Constants.Swerve.angleConversionPositionFactor)
            .velocityConversionFactor(Constants.Swerve.angleConversionVelocityFactor);
        
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                Constants.Swerve.rotationKP,
                Constants.Swerve.rotationKI,
                Constants.Swerve.rotationKD,
                Constants.Swerve.rotationKFF
            );

        config.voltageCompensation(Constants.Swerve.voltafeComp);

        rotationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        resetToAbsolute();
    }

    private void configDriveMotor(boolean driveInvert) {
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .inverted(driveInvert)
            .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
            .idleMode(Constants.Swerve.driveNeutralMode);

        config.encoder
            .positionConversionFactor(Constants.Swerve.driveConversionPositionFactor)
            .velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                Constants.Swerve.drivelKP,
                Constants.Swerve.driveKI,
                Constants.Swerve.driveKD,
                Constants.Swerve.driveKFF
            );

        config.voltageCompensation(Constants.Swerve.voltafeComp);

        rotationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder.setPosition(0.0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition(true).getValueAsDouble()-angleOffset.getDegrees());
    }
}