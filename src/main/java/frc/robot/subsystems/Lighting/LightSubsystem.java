package frc.robot.subsystems.Lighting;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public LightSubsystem() {
        // Add options to the chooser
        chooser.setDefaultOption("Idle", "IDLE");
        chooser.addOption("Off", "OFF");
        chooser.addOption("Automatic", "AUTO");
        chooser.addOption("Rainbow", "RGB");
        chooser.addOption("Team", "TEAM");

        // Add the chooser to Shuffleboard
        SmartDashboard.putData("LED Options", chooser);

        m_led = new AddressableLED(0);

        // Reuse buffer
        m_ledBuffer = new AddressableLEDBuffer(267);
        m_led.setLength(m_ledBuffer.getLength());


        // Set the data

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    boolean connected = false;
    @Override
    public void periodic() {
        if(!DriverStation.isDSAttached()) {
            connected = false;
            LEDPattern.solid(Color.kOrangeRed).breathe(Time.ofRelativeUnits(0.6, Seconds)).applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            return;
        }

        if(!connected) {
            connected = true;
            LEDPattern.solid(Color.kGreen).applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            return;
        }

        if(DriverStation.isAutonomousEnabled()) {
            LEDPattern.solid(Color.kAqua).breathe(Time.ofRelativeUnits(1, Seconds)).applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            return;
        }

        if(DriverStation.isTeleopEnabled()) {
            LEDPattern pattern = null;
            try {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    pattern = LEDPattern.solid(Color.kRed);
                } else {
                    pattern = LEDPattern.solid(Color.kBlue);
                }
            } catch (Exception e) {
                // TODO: handle exception
            }
            pattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            return;
        }

        // Retrieve the selected option
        String selectedMode = chooser.getSelected();

        if (selectedMode == null) return;
        LEDPattern pattern = null;

        // Perform actions based on the selected mode
        switch (selectedMode) {
            case "TEAM":
                try {
                    if (DriverStation.getAlliance().get() == Alliance.Red) {
                        pattern = LEDPattern.solid(Color.kRed);
                    } else {
                        pattern = LEDPattern.solid(Color.kBlue);
                    }
                } catch (Exception e) {
                    // TODO: handle exception
                }
                break;
            case "AUTO":
                pattern = LEDPattern.solid(Color.kAqua).breathe(Time.ofRelativeUnits(1, Seconds));
                break;
            case "RGB":
                Distance LED_SPACING = Meters.of(1.0 / 60);
                pattern = LEDPattern.rainbow(255,255).scrollAtAbsoluteSpeed(
                    InchesPerSecond.of(80), LED_SPACING
                );
                break;
            case "OFF":
                pattern = LEDPattern.solid(Color.kBlack);
                break;
            default:
                pattern = LEDPattern.solid(Color.kGreen).breathe(Time.ofRelativeUnits(3, Seconds));
                break;
        }

        // Apply the LED pattern to the data buffer
        pattern.applyTo(m_ledBuffer);

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
    }
}