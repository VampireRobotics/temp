package hardwareParts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CalculateTargetPower {

    // ===== valori primite din turela =====
    private double xTP = 0.0;
    private double yTP = 0.0;
    private double headingDegTP = 0.0;

    // ===== target (tx, ty) =====
    private double txTP = 0.0;
    private double tyTP = 0.0;

    // ===== distanta curenta (inch sau unitatea ta) =====
    public double distancePower = 0.0;

    // ===== rezultat power =====
    private double powerRaw = 0.0;
    private double powerFiltered = 0.0;

    // filtru (0..1). mai mic = mai smooth
    private double alpha = 0.8;

    // ===== lookup table din poza ta (distanta -> power) =====
    public static double[] distIn = {
            24,36, 48, 60, 72, 84, 96, 108, 120, 130, 140, 150
    };

    public static double[] power = {
            //0.51, 0.56, 0.555, 0.585, 0.605 , 0.655, 0.735, 0.617, 0.65, 0.661, 0.673,
            0.434, 0.438, 0.514, 0.524, 0.532, 0.564 , 0.587, 0.598, 0.63, 0.653, 0.665, 0.676
    };

    // ===== update chemat din turelaodometrieNew.loop() =====
    public void updateFromTurela(double x, double y, double headingDeg) {
        this.xTP = x;
        this.yTP = y;
        this.headingDegTP = headingDeg;
    }

    // setezi target si recalculezi tot (cheama asta in loop)
    public void updateTarget(double tx, double ty) {
        this.txTP = tx;
        this.tyTP = ty;

        // distanta live
        double dx = txTP - xTP;
        double dy = tyTP - yTP;
        distancePower = Math.hypot(dx, dy);

        // calc power din lookup + interpolare
        powerRaw = lookupInterpolated(distancePower);

        // filtru anti-jitter
        powerFiltered = powerFiltered + alpha * (powerRaw - powerFiltered);
    }

    // ===== getter-e =====
    public double getX() { return xTP; }
    public double getY() { return yTP; }
    public double getHeadingDeg() { return headingDegTP; }

    public double getPowerRaw() { return powerRaw; }
    public double getPower() { return powerFiltered; } // asta folosesti pe motor
    public double getDistance() { return distancePower; }

    public void setFilterAlpha(double a) {
        // clamp manual fara Range (ca sa ramana simplu)
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;
        this.alpha = a;
    }

    // ===== lookup + interpolare liniara =====
    private double lookupInterpolated(double d) {
        // clamp la capete
        if (d <= distIn[0]) return power[0];
        int last = distIn.length - 1;
        if (d >= distIn[last]) return power[last];

        // cauta intervalul
        for (int i = 0; i < last; i++) {
            double d0 = distIn[i];
            double d1 = distIn[i + 1];

            if (d >= d0 && d <= d1) {
                double p0 = power[i];
                double p1 = power[i + 1];

                // t in [0..1]
                double t = (d - d0) / (d1 - d0);
                return p0 + t * (p1 - p0);
            }
        }

        // fallback (nu ar trebui sa ajunga aici)
        return power[last];
    }
}
