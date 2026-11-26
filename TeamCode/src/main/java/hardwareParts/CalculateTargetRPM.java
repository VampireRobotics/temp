package hardwareParts;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CalculateTargetRPM {

    // ===== valori primite din turela =====
    private double xTP = 0.0;
    private double yTP = 0.0;
    private double headingDegTP = 0.0;

    // ===== target (tx, ty) =====
    private double txTP = 0.0;
    private double tyTP = 0.0;

    // ===== distanta curenta (inch sau unitatea ta) =====
    public double distanceRPM = 0.0;

    // ===== rezultat RPM =====
    private double rpmRaw = 0.0;
    private double rpmFiltered = 0.0;

    // filtru (0..1). mai mic = mai smooth
    private double alpha = 0.8;

    // ===== lookup table din poza ta (distanta -> RPM) =====
    public static double[] distIn = {
            24, 36, 48, 60, 72, 84, 96, 108, 120, 130, 140, 150
    };

    public static double[] rpm = {
            2000, 2270, 2500, 2650, 2825, 2885, 3130, 3260, 3410, 3575, 3650, 3800
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
        distanceRPM = Math.hypot(dx, dy);

        // calc RPM din lookup + interpolare
        rpmRaw = lookupInterpolated(distanceRPM);

        // filtru anti-jitter
        rpmFiltered = rpmFiltered + alpha * (rpmRaw - rpmFiltered);
    }

    // ===== getter-e =====
    public double getX() { return xTP; }
    public double getY() { return yTP; }
    public double getHeadingDeg() { return headingDegTP; }

    public double getRPMRaw() { return rpmRaw; }
    public double getRPM() { return rpmFiltered; } // asta folosesti pe motor
    public double getDistance() { return distanceRPM; }

    public void setFilterAlpha(double a) {
        // clamp manual fara Range (ca sa ramana simplu)
        if (a < 0.0) a = 0.0;
        if (a > 1.0) a = 1.0;
        this.alpha = a;
    }

    // ===== lookup + interpolare liniara =====
    private double lookupInterpolated(double d) {
        // clamp la capete
        if (d <= distIn[0]) return rpm[0];
        int last = distIn.length - 1;
        if (d >= distIn[last]) return rpm[last];

        // cauta intervalul
        for (int i = 0; i < last; i++) {
            double d0 = distIn[i];
            double d1 = distIn[i + 1];

            if (d >= d0 && d <= d1) {
                double r0 = rpm[i];
                double r1 = rpm[i + 1];

                // t in [0..1]
                double t = (d - d0) / (d1 - d0);
                return r0 + t * (r1 - r0);
            }
        }

        // fallback (nu ar trebui sa ajunga aici)
        return rpm[last];
    }
}
