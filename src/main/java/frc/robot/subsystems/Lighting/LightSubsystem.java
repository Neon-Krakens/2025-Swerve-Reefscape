package frc.robot.subsystems.Lighting;

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
    private int m_rainbowFirstPixelHue;

    public LightSubsystem() {
        // Add options to the chooser
        chooser.setDefaultOption("Off", "OFF");
        chooser.addOption("Automatic", "AUTO");
        chooser.addOption("Rainbow", "RGB");
        chooser.addOption("Team", "TEAM");

        // Add the chooser to Shuffleboard
        SmartDashboard.putData("LED Options", chooser);

        m_led = new AddressableLED(9);

        // Reuse buffer

        // Default to a length of 60, start empty output

        // Length is expensive to set, so only set it once, then just update data

        m_ledBuffer = new AddressableLEDBuffer(60);

        m_led.setLength(m_ledBuffer.getLength());


        // Set the data

        m_led.setData(m_ledBuffer);

        m_led.start();

        // Create an LED pattern that sets the entire strip to solid red
        LEDPattern red = LEDPattern.solid(Color.kRed);

        // Apply the LED pattern to the data buffer
        red.applyTo(m_ledBuffer);

        // Write the data to the LED strip
        m_led.setData(m_ledBuffer);
        
    }

    @Override
    public void periodic() {
        // Retrieve the selected option
        String selectedMode = chooser.getSelected();

        if (selectedMode == null)
            return;

        // Perform actions based on the selected mode
        switch (selectedMode) {
            case "TEAM":
                try {
                    if (DriverStation.getAlliance().get() == Alliance.Red) {
                        setLightColor(255, 0, 0);
                    } else {
                        setLightColor(0, 0, 255);
                    }
                } catch (Exception e) {
                    // TODO: handle exception
                }
                break;
            case "AUTO":
                setLightColor(0, 255, 255);
                break;
            case "RGB":
                // hue shift
                setLightColor(0, 255, 0);
                break;
            default:
                //
                setLightColor(0, 0, 0); // Off mode
                break;
        }

    }

    @Override
    public void simulationPeriodic() {

    }


    // Example method to set light color (implement based on your hardware)
    private void setLightColor(int r, int g, int b) {
        // for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
        //     m_ledBuffer.setLED(i, Color.kRed);
        // }
        // m_led.setData(m_ledBuffer);

        // Code to control your lights (e.g., send PWM signals or set values for LEDs)
        // System.out.printf("Setting lights to RGB(%d, %d, %d)%n", r, g, b);
    }
}