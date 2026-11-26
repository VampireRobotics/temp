package hardwareParts;

/** Ce are nevoie PolenSearchDrive de la o sursa de polen (limelightBB sau PolenDetector). */
public interface PolenSource {
    void update();
    boolean areBila();
    double getBilaTx();
    double getBilaTy();
    double getBilaTa();
    void modBile();
    void modBlueTag();
    void setScorAlegere(boolean on);
    void telemetrie();
}
