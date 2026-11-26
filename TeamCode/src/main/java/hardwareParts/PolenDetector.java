package hardwareParts;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Detectie polen cu pipeline-ul NEURONAL (Neural Detector) de pe Limelight.
 * Aplica filtre (clasa, confidence, arie, orizont, cadru vechi) si alege polenul cu scor + lock.
 * Acelasi API ca limelightBB (vezi PolenSource), deci PolenSearchDrive il poate folosi direct.
 */
@Config
public class PolenDetector implements PolenSource {

    // ---- pipeline-uri (indexul din UI-ul Limelight) ----
    public static int PIPELINE_POLEN = 3;   // Polen_NN (neuronal); 2 = Pollen_detecter (color), folosit de limelightBB
    public static int PIPELINE_TAG_BLUE = 1;
    // false = nu schimba pipeline-ul din cod: citeste ce ruleaza deja pe Limelight (il alegi tu din UI, pipeline 3)
    public static boolean AUTO_SWITCH = true;

    // ---- filtre ----
    public static String CLASS_NAME = "polen";   // "" = accepta orice clasa
    public static double CONF_MIN = 0.5;         // confidence minim
    public static double TA_MIN = 0.0;           // arie minima (%)
    public static double TA_MAX = 100.0;         // arie maxima (%)
    public static double TY_MAX_DEG = 100.0;     // respinge detectii cu ty mai mare (mai sus in cadru = "orizont"); 100 = oprit
    public static long STALE_MS = 500;           // ignoram rezultate mai vechi de atat (reteaua pe CPU poate avea FPS mic)

    // ---- scor + lock ----
    public static double SCOR_TX_DEG = 30;       // cat penalizeaza abaterea laterala
    public static long LOCK_MS = 500;            // cat tinem lock-ul fara sa mai vedem tinta
    public static double LOCK_RADIUS_DEG = 8;    // raza in care cautam tinta blocata

    public static int POLL_HZ = 100;

    private static final long SWITCH_RESEND_MS = 500;

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final ExecutorService io = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "LL-polen-switch");
        t.setDaemon(true);
        return t;
    });

    private int pipelineDorit = -1;
    private long lastSwitchMs = 0;
    private volatile boolean switchInFlight = false;
    private volatile boolean diagSwitchOk = false;
    private volatile int diagHttpCode = -1;
    private volatile String diagHttpErr = "-";
    private static final String LL_URL_FALLBACK = "http://172.29.0.1:5807";
    private String llUrl = LL_URL_FALLBACK;

    private boolean scorAlegere = true;
    private long lockSeenMs = 0;
    private double lockTx = 0, lockTy = 0;

    private boolean bilaVizibila = false;
    private double bilaTx = 0, bilaTy = 0, bilaTa = 0;

    // ---- diagnostic ----
    private int diagPipelineReal = -1;
    private int diagNrDetectii = 0, diagNrDupaFiltre = 0;
    private String diagClasa = "-";
    private double diagConf = 0;
    private double diagLatentaMs = 0;
    private long diagStalenessMs = 0;
    private String diagScoruri = "-";
    private boolean diagNull = true;

    public PolenDetector(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hw.get(Limelight3A.class, "limelight");
        llUrl = citesteBaseUrl();
        limelight.setPollRateHz(POLL_HZ);
        limelight.start();
    }

    private void setPipeline(int pipeline) {
        if (pipeline == pipelineDorit) return;
        pipelineDorit = pipeline;
        lockSeenMs = 0;
        if (AUTO_SWITCH) asyncSwitch(pipeline);
    }

    /** SDK-ul face POST /pipeline-switch cu connect-timeout de 100 ms si esueaza des: facem noi POST-ul (timeout mai mare). */
    private String citesteBaseUrl() {
        try {
            java.lang.reflect.Field f = Limelight3A.class.getDeclaredField("baseUrl");
            f.setAccessible(true);
            Object v = f.get(limelight);
            if (v instanceof String && !((String) v).isEmpty()) return (String) v;
        } catch (Exception ignored) {
            // ramanem pe fallback
        }
        return LL_URL_FALLBACK;
    }

    /** Comanda de schimbare pipeline pe thread separat (nu blocheaza bucla). */
    private void asyncSwitch(final int idx) {
        if (switchInFlight) return;
        switchInFlight = true;
        lastSwitchMs = System.currentTimeMillis();
        io.execute(() -> {
            java.net.HttpURLConnection c = null;
            try {
                c = (java.net.HttpURLConnection) new java.net.URL(llUrl + "/pipeline-switch?index=" + idx).openConnection();
                c.setRequestMethod("POST");
                c.setDoOutput(true);   // ca in SDK
                c.setRequestProperty("Content-Type", "application/json");
                c.setConnectTimeout(1000);
                c.setReadTimeout(1500);
                c.getOutputStream().close();
                int code = c.getResponseCode();
                diagHttpCode = code;
                diagHttpErr = "-";
                diagSwitchOk = (code == 200);
            } catch (Exception e) {
                diagHttpCode = -1;
                diagHttpErr = e.getClass().getSimpleName() + ": " + e.getMessage();
                diagSwitchOk = false;
            } finally {
                if (c != null) c.disconnect();
            }
            if (!diagSwitchOk) {
                // plan B: comanda din SDK
                try { diagSwitchOk = limelight.pipelineSwitch(idx); } catch (Exception ignored) { diagSwitchOk = false; }
            }
            switchInFlight = false;
        });
    }

    @Override public void modBile()    { setPipeline(PIPELINE_POLEN); }
    @Override public void modBlueTag() { setPipeline(PIPELINE_TAG_BLUE); }

    @Override
    public void setScorAlegere(boolean on) {
        scorAlegere = on;
        lockSeenMs = 0;
    }

    /** Scor de alegere: polen mare (aproape) si mai spre centru castiga. */
    public static double scor(double tx, double ta) {
        return ta / (1.0 + Math.abs(tx) / SCOR_TX_DEG);
    }

    @Override
    public void update() {
        bilaVizibila = false;

        LLResult res = limelight.getLatestResult();
        diagNull = res == null;
        diagPipelineReal = res != null ? res.getPipelineIndex() : -1;
        if (res == null) return;

        diagLatentaMs = res.getCaptureLatency() + res.getTargetingLatency();
        diagStalenessMs = res.getStaleness();

        long now = System.currentTimeMillis();
        // Limelight-ul poate fi pe alt pipeline decat dorim: retrimitem comanda (cu cooldown)
        if (AUTO_SWITCH) {
            if (pipelineDorit >= 0 && diagPipelineReal != pipelineDorit
                    && !switchInFlight && now - lastSwitchMs > SWITCH_RESEND_MS) {
                asyncSwitch(pipelineDorit);
            }
            if (diagPipelineReal != pipelineDorit) return;   // rezultat de pe alt pipeline
        }
        if (!res.isValid() || diagStalenessMs > STALE_MS) return;

        citestePolen(res, now);
    }

    private void citestePolen(LLResult res, long now) {
        List<LLResultTypes.DetectorResult> det = res.getDetectorResults();
        diagNrDetectii = det == null ? 0 : det.size();
        diagNrDupaFiltre = 0;
        if (det == null || det.isEmpty()) return;

        boolean oriceClasa = CLASS_NAME == null || CLASS_NAME.isEmpty();
        java.util.ArrayList<LLResultTypes.DetectorResult> ok = new java.util.ArrayList<>();
        StringBuilder sb = new StringBuilder();
        for (LLResultTypes.DetectorResult d : det) {
            if (!oriceClasa && !CLASS_NAME.equalsIgnoreCase(d.getClassName())) continue;
            if (d.getConfidence() < CONF_MIN) continue;
            double ta = d.getTargetArea();
            if (ta < TA_MIN || ta > TA_MAX) continue;
            if (d.getTargetYDegrees() > TY_MAX_DEG) continue;
            ok.add(d);
            sb.append(String.format(Locale.US, "[tx%.0f ta%.1f c%.2f s%.2f] ",
                    d.getTargetXDegrees(), ta, d.getConfidence(), scor(d.getTargetXDegrees(), ta)));
        }
        diagNrDupaFiltre = ok.size();
        diagScoruri = sb.length() == 0 ? "-" : sb.toString();
        if (ok.isEmpty()) return;

        LLResultTypes.DetectorResult best = null;
        if (scorAlegere) {
            // lock: ramanem pe acelasi polen cat timp il vedem aproape de unde era
            if (lockSeenMs != 0 && now - lockSeenMs <= LOCK_MS) {
                double bestDist = LOCK_RADIUS_DEG;
                for (LLResultTypes.DetectorResult d : ok) {
                    double dist = Math.hypot(d.getTargetXDegrees() - lockTx, d.getTargetYDegrees() - lockTy);
                    if (dist <= bestDist) { bestDist = dist; best = d; }
                }
            }
            if (best == null) {
                double bestScor = -1;
                for (LLResultTypes.DetectorResult d : ok) {
                    double s = scor(d.getTargetXDegrees(), d.getTargetArea());
                    if (s > bestScor) { bestScor = s; best = d; }
                }
            }
            lockSeenMs = now;
            lockTx = best.getTargetXDegrees();
            lockTy = best.getTargetYDegrees();
        } else {
            for (LLResultTypes.DetectorResult d : ok) {
                if (best == null || d.getTargetArea() > best.getTargetArea()) best = d;
            }
        }

        diagClasa = best.getClassName();
        diagConf = best.getConfidence();
        bilaVizibila = true;
        bilaTx = best.getTargetXDegrees();
        bilaTy = best.getTargetYDegrees();
        bilaTa = best.getTargetArea();
    }

    // ---------------- getters ----------------
    @Override public boolean areBila()  { return bilaVizibila; }
    @Override public double getBilaTx() { return bilaTx; }
    @Override public double getBilaTy() { return bilaTy; }
    @Override public double getBilaTa() { return bilaTa; }

    public int getPipeline()     { return pipelineDorit; }
    public double getLatentaMs() { return diagLatentaMs; }

    public void stop() {
        limelight.stop();
    }

    @Override
    public void telemetrie() {
        telemetry.addData("PD pipeline dorit/real", pipelineDorit + " / " + diagPipelineReal);
        telemetry.addData("PD switch ok / http cod", diagSwitchOk + " / " + diagHttpCode);
        telemetry.addData("PD http eroare", diagHttpErr);
        telemetry.addData("PD url", llUrl);
        telemetry.addData("PD connected / ms de la ultimul update", limelight.isConnected() + " / " + limelight.getTimeSinceLastUpdate());
        telemetry.addData("PD rezultat", diagNull ? "NULL" : "ok");
        telemetry.addData("PD latenta/vechime (ms)", String.format(Locale.US, "%.0f / %d", diagLatentaMs, diagStalenessMs));
        telemetry.addData("PD detectii brute/dupa filtre", diagNrDetectii + " / " + diagNrDupaFiltre);
        telemetry.addData("PD scoruri", diagScoruri);
        telemetry.addData("Polen", bilaVizibila);
        if (bilaVizibila) {
            telemetry.addData("Polen clasa/conf", diagClasa + " / " + String.format(Locale.US, "%.2f", diagConf));
            telemetry.addData("Polen tx/ty/ta", String.format(Locale.US, "%.1f / %.1f / %.2f", bilaTx, bilaTy, bilaTa));
        }
    }
}
