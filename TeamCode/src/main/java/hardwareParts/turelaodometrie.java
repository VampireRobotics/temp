package hardwareParts;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class turelaodometrie {

    public interface HeadingProvider {
        boolean isReady();
        double getHeadingRad();
    }

    private HeadingProvider headingProvider = null;

    public void setHeadingProvider(HeadingProvider provider) {
        this.headingProvider = provider;
    }



    // last valid heading (guard helper)
    private double lastHeadingDeg = 0.0;

    /** ia heading din provider, altfel ultima valoare valida */
    public double getHeadingDeg() {
        if (headingProvider != null && headingProvider.isReady()) {
            lastHeadingDeg = Math.toDegrees(headingProvider.getHeadingRad());
            return lastHeadingDeg;
        }

        return lastHeadingDeg;
    }

    // ================= HARDWARE =================
    public final Servo servoTurela;

    // ================= CONTROL =================
    public boolean turelaStart = false;

    // ================= STATE =================
    public double headingDeg = 0.0;
    public double error = 0.0;
    public double per = 0;
    public double headingc = 0;

    public double corection = 0.0;
    public double precizie = 5;

    public double servoposition = 0.5;
    public double posTurela = 0.5;

    public double targetdegrees = 138;

    // ✅ IMPORTANT: startangle este în GRADE (offset)
    public double startangle = 0;

    public turelaodometrie(HardwareMap hw) {
        servoTurela = hw.get(Servo.class, "servoTurela");
    }

    public void init() {
        posTurela = 0.5;
        servoposition = 0.5;
        servoTurela.setPosition(posTurela);
    }

    public void enableSearchOdometrie() {
        turelaStart = true;
    }

    public void disableSearchOdometrie() {
        turelaStart = false;
        servoTurela.setPosition(0.5);
    }

    public void loop() {
        if (!turelaStart) return;

        headingDeg = getHeadingDeg();

        // normalizare [0,360)
        per = Math.floor(headingDeg / 360.0);
        headingc = headingDeg - (per * 360.0);

        error = targetdegrees - headingc;

        if (Math.abs(error) >= precizie) {
            corection = (error / 42.0) * 0.3;
            servoposition = 0.5 + corection;
        }

        servoposition = Range.clip(servoposition, 0.2, 0.8);
        servoTurela.setPosition(servoposition);
    }
}