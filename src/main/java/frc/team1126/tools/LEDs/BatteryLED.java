// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126.tools.LEDs;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.team1126.Constants;
// import frc.robot.tools.MathTools;
// import frc.robot.Constants;
import frc.team1126.tools.MathTools;

public class BatteryLED extends AddressableLEDHelper {
    private int numLEDs;
    private int segmentLength;
    private int offset;

    private int redOffset, yellowOffset, greenOffset;

    // numLEDs has to be a multiple of 3
    public BatteryLED(int numLEDs) {
        super(numLEDs);
        this.numLEDs = numLEDs;

        segmentLength = numLEDs / 3;

        redOffset = segmentLength * 2;
        yellowOffset = segmentLength;
        greenOffset = 0;

        //SmartDashboard.putNumber("LedVoltageTest", 0);
    }

    @Override
    public void initialize(int offset) {
        this.offset = offset;
    }

    MedianFilter filter = new MedianFilter(10);
    @Override
    public AddressableLEDBuffer writeData(AddressableLEDBuffer buffer) {
        double voltage = filter.calculate(RobotController.getBatteryVoltage());
        //double voltage = SmartDashboard.getNumber("LedVoltageTest", 0);
    
        int numberOfLeds = (int)MathTools.map(voltage, Constants.BatteryMonitor.MINVOLTAGE,
                Constants.BatteryMonitor.MAXVOLTAGE, 1, numLEDs);

        for (int i = offset; i < segmentLength + offset; i++) {
            // Red
            buffer.setLED(i + greenOffset,
            (i + greenOffset < numberOfLeds
                    ? super.setPercentBrightness(Color.kRed, Constants.BatteryMonitor.BRIGHTNESS)
                    : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));

            // Yellow
            buffer.setLED(i + yellowOffset,
                    (i + yellowOffset < numberOfLeds
                            ? super.setPercentBrightness(Color.kYellow, Constants.BatteryMonitor.BRIGHTNESS)
                            : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
        
            // Green
            buffer.setLED(i + redOffset,
            (i + redOffset < numberOfLeds
                    ? super.setPercentBrightness(Color.kGreen, Constants.BatteryMonitor.BRIGHTNESS)
                    : super.setPercentBrightness(Color.kBlack, Constants.BatteryMonitor.BRIGHTNESS)));
        }
       
        return buffer;
    }
}