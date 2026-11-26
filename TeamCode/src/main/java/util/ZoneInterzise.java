package util;

import com.acmerobotics.dashboard.config.Config;

/**
 * Zone de pe teren (coordonate PedroPathing, inch) pe care robotul NU trebuie sa le atinga:
 * doua zone: x=0 si x=46, ambele pe y 51-91 (x orizontal, y vertical).
 *
 * O bila e "blocata" daca pozitia ei estimata sau drumul robotului pana la ea trece prin
 * una din zone (marite cu MARGINE_IN ca sa tinem cont de latimea robotului).
 */
@Config
public class ZoneInterzise {

    public static boolean ENABLED = true;

    // [xMin, xMax, yMin, yMax]
    // x = orizontal, y = vertical. Doua zone, ambele cu y intre 51 si 91:
    //   A: la x = 0, B: la x = 46 (latimea reala vine din MARGINE_IN)
    public static double A_X_MIN = 0,  A_X_MAX = 0;
    public static double B_X_MIN = 46, B_X_MAX = 46;
    public static double BARA_Y_MIN = 51, BARA_Y_MAX = 91;

    /** Cat marim fiecare zona (jumatate din latimea robotului + siguranta). */
    public static double MARGINE_IN = 8;

    /** Marja pe latura de jos (y mic) a fiecarei zone; mai mica ca sa nu evite prea devreme. */
    public static double MARGINE_JOS_IN = 4;

    /** Distanta estimata pana la bila: DIST_K / sqrt(ta). De calibrat pe robot. */
    public static double DIST_K = 15;
    public static double DIST_MAX_IN = 60;

    private static double[][] zone() {
        return new double[][]{
                {A_X_MIN, A_X_MAX, BARA_Y_MIN, BARA_Y_MAX},   // zona A (x=0, y 51-91)
                {B_X_MIN, B_X_MAX, BARA_Y_MIN, BARA_Y_MAX},   // zona B (x=46, y 51-91)
        };
    }

    /**
     * @param x,y,headingRad pozitia robotului pe teren (heading 0 = spre +X)
     * @param txDeg          unghiul bilei fata de centrul camerei (pozitiv = la dreapta)
     * @param ta             aria bilei in cadru (%), folosita pentru distanta
     * @return true daca bila sau drumul spre ea intersecteaza o zona interzisa
     */
    public static boolean estePeBara(double x, double y, double headingRad, double txDeg, double ta) {
        if (!ENABLED) return false;

        double dist = ta > 0.01 ? Math.min(DIST_MAX_IN, DIST_K / Math.sqrt(ta)) : DIST_MAX_IN;
        double bearing = headingRad - Math.toRadians(txDeg);   // tx pozitiv = dreapta = in sens orar
        double bx = x + dist * Math.cos(bearing);
        double by = y + dist * Math.sin(bearing);

        return segmentBlocat(x, y, bx, by);
    }

    /** Cat de departe in fata verificam daca robotul intra intr-o zona (frana inainte de zona). */
    public static double LOOKAHEAD_IN = 8;

    /**
     * Pastreaza robotul in afara zonelor mereu (independent de bile): daca mersul inainte pe
     * directia curenta ar intra in zona (marita cu margini), returneaza false.
     * Daca robotul e deja in zona, permite doar miscarea care il scoate afara.
     */
    public static boolean inainteLiber(double x, double y, double headingRad) {
        if (!ENABLED) return true;
        double bx = x + LOOKAHEAD_IN * Math.cos(headingRad);
        double by = y + LOOKAHEAD_IN * Math.sin(headingRad);
        return !segmentBlocat(x, y, bx, by);
    }

    private static boolean segmentBlocat(double x, double y, double bx, double by) {
        for (double[] z : zone()) {
            double x0 = z[0] - MARGINE_IN, x1 = z[1] + MARGINE_IN;
            double y0 = z[2] - MARGINE_JOS_IN, y1 = z[3] + MARGINE_IN;

            if (bx >= x0 && bx <= x1 && by >= y0 && by <= y1) return true;

            // daca robotul e deja in zona marita, verificam doar bila (altfel nu s-ar mai putea misca)
            boolean robotInZona = x >= x0 && x <= x1 && y >= y0 && y <= y1;
            if (!robotInZona && segmentIntersecteazaDreptunghi(x, y, bx, by, x0, x1, y0, y1)) return true;
        }
        return false;
    }

    /** Liang-Barsky: segmentul (ax,ay)-(bx,by) intersecteaza dreptunghiul? */
    private static boolean segmentIntersecteazaDreptunghi(double ax, double ay, double bx, double by,
                                                          double x0, double x1, double y0, double y1) {
        double dx = bx - ax, dy = by - ay;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {ax - x0, x1 - ax, ay - y0, y1 - ay};
        double t0 = 0, t1 = 1;
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double r = q[i] / p[i];
                if (p[i] < 0) { if (r > t1) return false; if (r > t0) t0 = r; }
                else          { if (r < t0) return false; if (r < t1) t1 = r; }
            }
        }
        return true;
    }
}
