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
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax rotationMotor; // Rotation Neo Motor Controller
    private SparkMax driveMotor; // Drive Neo Motor Controller

    private RelativeEncoder rotationEncoder; // Encoder from the NEO motor
    private RelativeEncoder driveEncoder; // Encoder from the NEO motor
    
    private CANcoder absoluteEncoder; // CANcoder located above the swerve module

    private final SparkClosedLoopController driveController; // Very precise way of controlling Drive SparkMax (PID)
    private final SparkClosedLoopController rotationController; // Very precise way of controlling Rotation SparkMax (PID)

    public SwerveModule(int moduleNumber, ModuleConfig moduleConfig){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConfig.angleOffset;

        // CANCoder config
        this.absoluteEncoder = new CANcoder(moduleConfig.canCoderID);
        Swerve.configureCANcoder(absoluteEncoder, moduleConfig.canCoderInvert);

        // rotation motor config
        this.rotationMotor = new SparkMax(moduleConfig.rotationMotorID, MotorType.kBrushless);
        this.rotationEncoder = rotationMotor.getEncoder();
        this.rotationController = rotationMotor.getClosedLoopController();
        configRotationMotor(moduleConfig.rotationInvert);

        // drive Motor Config
        this.driveMotor = new SparkMax(moduleConfig.driveMotorID, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        this.driveController = driveMotor.getClosedLoopController();
        configDriveMotor(moduleConfig.driveInvert);

        this.lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        // setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setAngle(double targetAngle) {
        // Convert CanCoder output to degrees (-0.5 to 0.5 scaled to -180 to 180)
        double currentAngle = getCanCoder().getDegrees() * 360;
        // Normalize current angle to [0, 360)
        currentAngle = (currentAngle % 360 + 360) % 360;
        // Normalize target angle to [0, 360)
        targetAngle = (targetAngle % 360 + 360) % 360;
        // Calculate the shortest path to the target angle
        double deltaAngle = ((targetAngle - currentAngle + 540) % 360) - 180;
        // Calculate the desired angle
        double desiredAngle = currentAngle + deltaAngle;
        // Set the motor controller to the desired angle
        rotationController.setReference(desiredAngle, ControlType.kPosition);
    }

    private void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
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

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                desiredState.speedMetersPerSecond,
                ControlType.kVelocity
            );
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))? lastAngle: desiredState.angle;

        rotationController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(rotationEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition(true).getValueAsDouble());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public double getPosition() {
        return driveEncoder.getPosition();
    }
}