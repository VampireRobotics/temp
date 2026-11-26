package senzori;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SenzorCuloare {

    public enum CuloareArtefact {
        PURPLE,
        GREEN,
        NONE
    }

    private final ColorSensor colorSensor;
    private final Telemetry telemetry;
    private final float[] hsv = new float[3];

    public static int INTENSITY_THRESHOLD = 800;
    public static float GREEN_H_MIN = 130f;
    public static float GREEN_H_MAX = 190f;
    public static float PURPLE_H_MIN = 190f;
    public static float PURPLE_H_MAX = 280f;

    public SenzorCuloare(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public CuloareArtefact detecteazaCuloare() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int intensity = r + g + b;

        Color.RGBToHSV(r, g, b, hsv);
        float h = hsv[0];

        CuloareArtefact culoare = CuloareArtefact.NONE;

        if (intensity < INTENSITY_THRESHOLD) {
            culoare = CuloareArtefact.NONE;
        } else {
            if (h >= GREEN_H_MIN && h <= GREEN_H_MAX) {
                culoare = CuloareArtefact.GREEN;
            }
            else if (h >= PURPLE_H_MIN && h <= PURPLE_H_MAX) {
                culoare = CuloareArtefact.PURPLE;
            } else {
                culoare = CuloareArtefact.NONE;
            }
        }

//        telemetry.addData("R", r);
//        telemetry.addData("G", g);
//        telemetry.addData("B", b);
//        telemetry.addData("Intensity", intensity);
//        telemetry.addData("Hue", h);
//        telemetry.addData("Detect", culoare);
        return culoare;
    }
}
