package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainLEDs extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    static int hue = 80;
    static int value = 255;

    // Variables for noise things
    static int fluctuationRange = 15;
    static double fluctuationRate = 0.01;
    static double offset = 0;
    static double baseHue = 10;
    static double hueChangeRate = 0;

    // Hue Pulse Variables
    int pulseColors[] = { 10, 100 };
    int pulseIndex = 0;
    double updatePoint = 0.1;
    double updateRate = 0.1;

    /** Creates a new LEDs. */
    public DriveTrainLEDs() {
        m_led = new AddressableLED(1);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(52);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        DriveTrainLEDs.hue = 80;
    }

    public static int getHue() {
        return hue;
    }

    public static void setHue(int hue) {
        DriveTrainLEDs.hue = hue;
    }

    public static void setHueLerp(double[] range, double lerpVal) {
        DriveTrainLEDs.hue = (int) Lerp(range, lerpVal) % 181;
    }

    public static void setValue(int value) {
        DriveTrainLEDs.value = value;
    }

    private static double Lerp(double[] range, double lerpVal) {
        return (range[1] - range[0]) * lerpVal + range[0];
    }

    private int solidNoiseFluctuation() {
        double noiseVal = OpenSimplex.noise2_ImproveX(1934173838, 0, offset);
        noiseVal *= fluctuationRange;
        noiseVal += hue;
        noiseVal = (noiseVal + 181) % 181;

        offset += fluctuationRate;
        if (Math.abs(offset) > 100000) {
            fluctuationRate *= -1;
        }

        return (int) noiseVal;
    }

    double cyclerCounter = 0;

    private void colorCycler() {
        int change = 40;
        double rate = 0.25;// smaller is faster
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hue, 255, value);
            if (cyclerCounter / rate > i) {
                m_ledBuffer.setHSV(i, (hue + change) % 181, 255, value);
            }
        }

        cyclerCounter += 0.1;
        if (cyclerCounter >= m_ledBuffer.getLength() * rate) {
            cyclerCounter = 0;
            hue = (hue + change) % 181;
        }
    }

    private void noise() {
        double scale = 0.05;
        for (var x = 0; x < m_ledBuffer.getLength(); x++) {
            double noiseVal = OpenSimplex.noise2_ImproveX(1934173838, x * scale, offset);
            int h = (int) ((baseHue + 181 + noiseVal * fluctuationRange) % 181);
            m_ledBuffer.setHSV(x, h, 255, value);
        }

        // baseHue += hueChangeRate;
        // baseHue %= 181;

        offset += fluctuationRate;
        if (Math.abs(offset) > 100000) {
            fluctuationRate *= -1;
        }
    }

    @Override
    public void periodic() {
        // Gives the hue color a bit of fluctuation using OpenSimplex noise
        // int h = solidNoiseFluctuation();

        // Set all LEDs to the hue color
        // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // m_ledBuffer.setHSV(i, hue, 255, value);
        // }

        // noise();
        colorCycler();
        m_led.setData(m_ledBuffer);

    }
}