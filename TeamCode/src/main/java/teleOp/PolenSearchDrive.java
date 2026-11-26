package teleOp;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.math.Pose;

import hardwareParts.PolenSource;
import util.ZoneInterzise;

/**
 * Algoritmii de detectie + urmarire polen (bila) cu Limelight (pipeline 2), plus CAUTARE:
 * daca nu mai vede nicio bila un timp scurt (SEARCH_AFTER_MS), se invarte in jur in pulsuri
 * (rotește SEARCH_ROTATE_MS, sta SEARCH_PAUSE_MS ca Limelight-ul sa apuce sa detecteze),
 * schimband sensul la fiecare SWEEP_PULSES pulsuri ca sa ramana in zona. Cand vede o bila,
 * trece automat pe urmarire.
 *
 * Bazat pe DriveToPolen (neschimbat). Folosire: update() in fiecare loop, apoi daca
 * areComanda() -> sasiu.Drive(getForward(), getStrafe(), getTurn()).
 */
@Config
public class PolenSearchDrive {

    // ---- urmarire (la fel ca DriveToPolen) ----
    public static double KP_TURN = 0.03;
    public static double MAX_TURN = 0.5;
    public static double TX_DEADBAND = 1.5;    // grade

    public static double FORWARD_SPEED = 0.7;
    public static double TURN_SLOWDOWN_TX = 30;
    public static long LOST_HOLD_MS = 250;     // cat timp tinem ultima tinta dupa ce pierdem bila

    public static double ARRIVE_TA = 6.0;      // aria (%) la care consideram ca am ajuns la bila
    public static double PUSH_SPEED = 0.25;
    public static long PUSH_MS = 600;

    public static int INTAKE_RPM = 1600;

    // ---- cautare ----
    public static boolean SEARCH_ENABLED = true;
    public static long SEARCH_AFTER_MS = 500;  // fara bila cat timp inainte sa porneasca cautarea (> LOST_HOLD_MS)
    public static double SEARCH_TURN = 0.3;    // putere rotire in cautare (+ / - = sensul de start)
    public static long SEARCH_ROTATE_MS = 250; // cat se roteste intr-un puls
    public static long SEARCH_PAUSE_MS = 300;  // cat sta pe loc sa detecteze
    public static int SWEEP_PULSES = 6;        // dupa cate pulsuri inverseaza sensul

    private final PolenSource ll;

    private boolean activ = false;
    private boolean comanda = false;
    private boolean bilaVizibila = false;
    private boolean cautare = false;
    private double forward = 0, strafe = 0, turn = 0;
    private long pushStartMs = 0;
    private long enableMs = 0;
    private long lastSeenMs = 0;
    private long searchStartMs = 0;
    private double lastTx = 0, lastTa = 0;

    /** Furnizeaza pozitia robotului pe teren (Pedro); null = nu stim, nu filtram bilele. */
    public interface PoseProvider {
        Pose getPose();
    }

    private PoseProvider poseProvider = null;
    private boolean bilaBlocata = false;

    public PolenSearchDrive(PolenSource ll) {
        this.ll = ll;
    }

    public void setPoseProvider(PoseProvider p) { this.poseProvider = p; }
    /** true daca ultima bila vazuta a fost ignorata pentru ca e pe/langa o bara de cage. */
    public boolean bilaBlocata() { return bilaBlocata; }

    private boolean estePeBara() {
        if (poseProvider == null) return false;
        Pose p = poseProvider.getPose();
        if (p == null) return false;
        return ZoneInterzise.estePeBara(p.x(), p.y(), p.heading(), ll.getBilaTx(), ll.getBilaTa());
    }

    public void enable() {
        activ = true;
        pushStartMs = 0;
        lastSeenMs = 0;
        searchStartMs = 0;
        cautare = false;
        enableMs = System.currentTimeMillis();
        ll.modBile();
        ll.setScorAlegere(true);
    }

    public void disable() {
        ll.setScorAlegere(false);
        activ = false;
        comanda = false;
        cautare = false;
        forward = strafe = turn = 0;
        pushStartMs = 0;
        searchStartMs = 0;
    }

    public boolean isActiv() { return activ; }

    public void update() {
        comanda = false;
        cautare = false;
        forward = strafe = turn = 0;
        bilaVizibila = false;
        zonaFrana = false;
        if (!activ) return;

        ll.update();
        long t = System.currentTimeMillis();
        bilaBlocata = ll.areBila() && estePeBara();
        if (bilaBlocata) lastSeenMs = 0;   // ignoram bila: fara "hold" spre ea
        if (ll.areBila() && !bilaBlocata) {
            lastSeenMs = t;
            lastTx = ll.getBilaTx();
            lastTa = ll.getBilaTa();
            searchStartMs = 0;
        } else {
            long lastActivity = Math.max(lastSeenMs, enableMs);
            long lost = t - lastActivity;
            pushStartMs = 0;

            if (lastSeenMs != 0 && t - lastSeenMs <= LOST_HOLD_MS) {
                // pierdere scurta: continuam spre ultima tinta (cod de mai jos)
            } else if (SEARCH_ENABLED && lost >= SEARCH_AFTER_MS) {
                cauta(t);
                return;
            } else {
                return;   // stam pe loc si ne uitam
            }
        }
        bilaVizibila = true;

        double tx = lastTx;
        double ta = lastTa;

        if (Math.abs(tx) > TX_DEADBAND) {
            turn = clamp(KP_TURN * tx, -MAX_TURN, MAX_TURN);
        }

        long now = System.currentTimeMillis();
        if (ta >= ARRIVE_TA) {
            // am ajuns: mai impingem putin ca sa o inghita intake-ul
            if (pushStartMs == 0) pushStartMs = now;
            if (now - pushStartMs <= PUSH_MS) {
                forward = PUSH_SPEED;
            }
        } else {
            pushStartMs = 0;
            double factor = 1.0 - Math.min(1.0, Math.abs(tx) / TURN_SLOWDOWN_TX);
            forward = FORWARD_SPEED * factor;
        }
        comanda = true;

        // paza: nu intram niciodata in zonele interzise, indiferent de bile (ramane doar rotirea)
        if (forward > 0 && poseProvider != null) {
            Pose p = poseProvider.getPose();
            if (p != null && !ZoneInterzise.inainteLiber(p.x(), p.y(), p.heading())) {
                forward = 0;
                zonaFrana = true;
            }
        }
    }

    private boolean zonaFrana = false;
    /** true daca mersul inainte a fost oprit ca sa nu intram intr-o zona interzisa. */
    public boolean zonaFrana() { return zonaFrana; }

    /** Rotire in pulsuri: roteste, apoi sta pe loc ca Limelight-ul sa poata detecta clar. */
    private void cauta(long t) {
        if (searchStartMs == 0) searchStartMs = t;
        cautare = true;
        comanda = true;

        long cycle = Math.max(1, SEARCH_ROTATE_MS + SEARCH_PAUSE_MS);
        long elapsed = t - searchStartMs;
        long pulse = elapsed / cycle;
        long phase = elapsed % cycle;

        if (phase < SEARCH_ROTATE_MS) {
            boolean inverseaza = SWEEP_PULSES > 0 && ((pulse / SWEEP_PULSES) % 2 == 1);
            turn = inverseaza ? -SEARCH_TURN : SEARCH_TURN;
        }
    }

    /** true cand avem comenzi de drive de aplicat (urmarire sau cautare). */
    public boolean areComanda()    { return comanda; }
    public boolean bilaVizibila()  { return bilaVizibila; }
    public boolean seCauta()       { return cautare; }
    /** true cat timp trebuie pornit intake-ul (bila vizibila). */
    public boolean vreaIntake()    { return activ && bilaVizibila; }

    public double getForward() { return forward; }
    public double getStrafe()  { return strafe; }
    public double getTurn()    { return turn; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
