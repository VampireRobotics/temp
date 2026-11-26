package senzori;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MagneticLimitSwitch {
    public DigitalChannel limitSwitch;

    // Constructor: primește hardwareMap și numele senzorului în configurație
    public MagneticLimitSwitch(HardwareMap hardwareMap, String sensorName) {
        this.limitSwitch = hardwareMap.get(DigitalChannel.class, sensorName);
        this.limitSwitch.setMode(DigitalChannel.Mode.INPUT);  // Setează modul de citire digitală
    }

    // Returnează true dacă senzorul este activat (circuit închis)
    public boolean isPressed() {
        return !limitSwitch.getState();
    }
}
