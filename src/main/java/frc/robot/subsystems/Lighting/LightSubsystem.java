package frc.robot.subsystems.Lighting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {

    private final SendableChooser<String> chooser = new SendableChooser<>();

    public LightSubsystem() {
        // Add options to the chooser
        chooser.setDefaultOption("Off", "OFF");
        chooser.addOption("Automatic", "AUTO");
        chooser.addOption("Rainbow", "RGB");
        chooser.addOption("Team", "TEAM");

        // Add the chooser to Shuffleboard
        SmartDashboard.putData("LED Options", chooser);
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
        // Code to control your lights (e.g., send PWM signals or set values for LEDs)
        // System.out.printf("Setting lights to RGB(%d, %d, %d)%n", r, g, b);
    }
}