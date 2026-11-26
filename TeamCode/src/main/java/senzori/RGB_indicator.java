package senzori;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGB_indicator {
    public Servo light_fata = null;
    public Servo light_spate = null;
    public HardwareMap hardwareMap;

    public RGB_indicator(HardwareMap hardwareMap) {
        this.light_fata = hardwareMap.get(Servo.class, "light_fata");
        this.light_spate = hardwareMap.get(Servo.class, "light_spate");
        this.hardwareMap = hardwareMap;
    }

    public void light_white() {
        light_fata.setPosition(1.0);
        light_spate.setPosition(1.0);
    }

    public void light_red(){
        light_fata.setPosition(0.280);
        light_spate.setPosition(0.280);
    }
    public void light_green(){
        light_fata.setPosition(0.5);
        light_spate.setPosition(0.5);
    }
    public void light_purple(){
        light_fata.setPosition(0.722);
        light_spate.setPosition(0.722);
    }
}
