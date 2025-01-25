package frc.team1126.tools.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class SetLEDColor extends AddressableLEDHelper {
    private int numLEDs;

    public SetLEDColor(int num) {
        super(num);
    }
    
    public static void setRed(AddressableLEDBuffer buffer) {
        LEDPattern red = LEDPattern.solid(Color.kRed);
        red.applyTo(buffer);

    }
}
