package teleOp;

import static hardwareParts.ShooterPSV.ANGLE_ADJUST;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import hardwareParts.CalculateTargetPower;
import hardwareParts.CalculateTargetRPM;
import hardwareParts.IntakeDecode;
import hardwareParts.Sasiu;
import hardwareParts.ShooterPSV;
import hardwareParts.TurelaServo;
import hardwareParts.PolenDetector;
import hardwareParts.PolenSource;
import hardwareParts.limelightBB;
import hardwareParts.turelaodometrieNew;
import localization.PedroLocalization;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import util.SharedAutoData;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="teleopBioBuzz")
public class teleopBioBuzz extends LinearOpMode {

    public Sasiu sasiu;
    public IntakeDecode intake;
    public ShooterPSV shooter;

    public turelaodometrieNew turela;
    public CalculateTargetRPM calcRPM;       // pentru shooter
    private boolean useDynamicPower = false;

    private Limelight3A limelightPose;
    private boolean limelightPoseStarted = false;
    private static final double M_TO_IN = 39.3700787;

    private Follower follower;
    private PedroLocalization localization;
    private Pose startPoseTeleop;

    long burstStartMs = 0;
    long lastShotMs = 0;

    static final long SHOT_PERIOD_MS = 50;
    static final long FIRST_SHOT_DELAY_MS = 100;

    private boolean useLimelightTurret = false;
    private boolean limelightStarted = false;

    private boolean prevLeftStickBtn = false;

    private PolenSource llBB;
    // true = detector neuronal (PolenDetector), false = pipeline color (limelightBB)
    public static boolean USE_NEURAL_DETECTOR = false;
    private PolenSearchDrive driveToPolen;

    @Override
    public void runOpMode() throws InterruptedException {

        // ===== Hardware init =====
        sasiu    = new Sasiu(hardwareMap, telemetry);
        intake   = new IntakeDecode(hardwareMap, telemetry);
        calcRPM   = new CalculateTargetRPM();
        shooter   = new ShooterPSV(hardwareMap, telemetry, calcRPM);

        turela   = new turelaodometrieNew(hardwareMap);
        turela.setTelemetry(telemetry);

        // ===== Limelight pose =====
        try {
            limelightPose = hardwareMap.get(Limelight3A.class, "limelight");
            limelightPose.setPollRateHz(100);
            limelightPose.start();
            limelightPose.pipelineSwitch(0);
            limelightPoseStarted = true;
        } catch (Exception e) {
            limelightPose = null;
            limelightPoseStarted = false;
        }

        // ===== Localization =====
        try {
            follower = Constants.create(hardwareMap);

            // start mereu din (0,0) = coltul stanga-jos, cu fata spre +X (ca in citirePozitii)
            startPoseTeleop = new Pose(0, 0, 0);

            follower.setPose(startPoseTeleop);

            localization = new PedroLocalization(follower);

        } catch (Exception e) {
            follower = null;
            localization = null;
        }

        if (localization != null) localization.setTelemetry(this.telemetry);

        // ===== Pose provider for odo turret from PedroLocalization =====
        turela.setPoseProvider(new turelaodometrieNew.PoseProvider() {
            @Override public boolean isReady() {
                return localization != null && localization.getPose() != null;
            }
            @Override public double getHeadingRad() {
                return localization.getHeadingRad();
            }
            @Override public double getX() {
                Pose p = (localization != null) ? localization.getPose() : null;
                return (p != null) ? p.x() : 0.0;
            }
            @Override public double getY() {
                Pose p = (localization != null) ? localization.getPose() : null;
                return (p != null) ? p.y() : 0.0;
            }
        });

        turela.init();
        shooter.init();

        turela.targetx = 10.673684210526316;
        turela.targety = 127.44210526315788;

        // ===== INIT LOOP =====
        while (!isStarted() && !isStopRequested()) {
            if (localization != null) localization.update();

            if (!useLimelightTurret) {
                calcRPM.updateFromTurela(turela.currentx, turela.currenty, turela.headingWrappedDeg);
                calcRPM.updateTarget(turela.targetx, turela.targety);
            }

            if (!useLimelightTurret && useDynamicPower) {
                shooter.setTargetRPM(calcRPM.getRPM());
            }

            shooter.read();
            shooter.update();
            shooter.write();

            telemetry.addLine("=== INIT LOOP (TBE) ===");
            telemetry.update();
        }

        waitForStart();

        shooter.enableAutoAngle(false);
        shooter.autocircleAngle(false);
        shooter.off();
        shooter.setTargetRPM(0.0);

        useLimelightTurret = false;
        limelightStarted = false;
        useDynamicPower = false;

        turela.disableSearchOdometrie();

        prevLeftStickBtn = gamepad1.left_stick_button;

        // ===== MAIN LOOP =====
        while (opModeIsActive()) {


            if (localization != null) localization.update();

            if (limelightPoseStarted && limelightPose != null && localization != null) {
                limelightPose.updateRobotOrientation(localization.getHeadingDeg());
            }

            // ===== MODE BUTTONS =====

            // CROSS = START shooter + ODO turret + dynamic power ON
            if (gamepad1.crossWasPressed()) {
                useLimelightTurret = false;

                turela.disableTurelaFixa();
                turela.enableSearchOdometrie();
                useDynamicPower = true;

                shooter.on();
                shooter.enableAutoAngle(true);
                shooter.autocircleAngle(false);
            }

            // CIRCLE = pose reset
            if (gamepad1.circleWasPressed()) {
                useLimelightTurret = false;

                Pose circlePose = new Pose(17.121495327102803, 65, Math.toRadians(180));//67.4392523364486

                if (localization != null) {
                    localization.resetPose(circlePose);
                } else if (follower != null) {
                    follower.setPose(circlePose);
                }
            }

            // Toggle limelight turret with left stick button edge
            boolean leftStickNow = gamepad1.left_stick_button;
            boolean leftStickEdge = leftStickNow && !prevLeftStickBtn;
            prevLeftStickBtn = leftStickNow;

            if (leftStickEdge) {
                useLimelightTurret = !useLimelightTurret;

                if (useLimelightTurret) {
                    turela.disableSearchOdometrie();
                    turela.disableTurelaFixa();
                    useDynamicPower = false;

                    if (!limelightStarted) {
                        limelightStarted = true;
                    }
                } else {

                    turela.disableTurelaFixa();
                    turela.enableSearchOdometrie();
                }
            }

            // SQUARE = STOP / reset
            if (gamepad1.squareWasPressed()) {
                turela.disableTurelaFixa();
                turela.disableSearchOdometrie();
                turela.init();

                shooter.enableAutoAngle(false);
                shooter.autocircleAngle(false);

                useDynamicPower = false;
                shooter.off();
            }

            // manual turret trim
            if (gamepad1.dpadLeftWasPressed())  turela.trimLeft();
            if (gamepad1.dpadRightWasPressed()) turela.trimRight();
            if (gamepad1.dpadUpWasPressed())    turela.resetTrim();

            // DPAD DOWN = EMERGENCY turret fix
            if (gamepad1.dpadDownWasPressed()) {
                useLimelightTurret = false;

                turela.enableTurelaFixa();
            }

            // updateaza calcRPM cu pose-ul curent din turela
            if (!useLimelightTurret) {
                calcRPM.updateFromTurela(turela.currentx, turela.currenty, turela.headingWrappedDeg);
                calcRPM.updateTarget(turela.targetx, turela.targety);
            }

            // dynamic shooter RPM before PSV control
            if (!useLimelightTurret && useDynamicPower) {
                shooter.setTargetRPM(calcRPM.getRPM());
            }

            // shooter PID
            shooter.read();
            shooter.update();
            shooter.write();

            // ===== DRIVE TO POLEN (toggle: right stick button) =====
            if (gamepad1.rightStickButtonWasPressed()) {
                if (driveToPolen != null && driveToPolen.isActiv()) {
                    driveToPolen.disable();
                    if (llBB != null) llBB.modBlueTag();   // revenim la pipeline-ul de tag
                } else {
                    try {
                        if (llBB == null) {
                            llBB = USE_NEURAL_DETECTOR
                                    ? new PolenDetector(hardwareMap, telemetry)
                                    : new limelightBB(hardwareMap, telemetry);
                            driveToPolen = new PolenSearchDrive(llBB);
                            driveToPolen.setPoseProvider(() -> localization != null ? localization.getPose() : null);
                        }
                        driveToPolen.enable();
                    } catch (Exception e) {
                        llBB = null;
                        driveToPolen = null;
                    }
                }
            }
            boolean polenActiv = driveToPolen != null && driveToPolen.isActiv();
            if (polenActiv) driveToPolen.update();

            // ===== SASIU =====
            boolean stickMiscat = Math.abs(gamepad1.left_stick_x) > 0.1
                    || Math.abs(gamepad1.left_stick_y) > 0.1
                    || Math.abs(gamepad1.right_stick_x) > 0.1;

            if (polenActiv && driveToPolen.areComanda() && !stickMiscat) {
                sasiu.Drive(driveToPolen.getForward(), driveToPolen.getStrafe(), driveToPolen.getTurn());
            } else {
                sasiu.Drive(-gamepad1.left_stick_y, +gamepad1.left_stick_x, +gamepad1.right_stick_x);
            }

            // ===== INTAKE =====
            if (gamepad1.right_trigger > 0.1) intake.setIntakeRPM(1600);

            else if (gamepad1.left_trigger > 0.1) intake.setIntakeRPM(-1600);
            else if (polenActiv && driveToPolen.vreaIntake()) intake.setIntakeRPM(PolenSearchDrive.INTAKE_RPM);
            else intake.stop();

            // ===== BURST SHOOTING =====
            if (gamepad1.rightBumperWasPressed()) {
                shooter.beginShootingBurst();
                burstStartMs = System.currentTimeMillis();
                lastShotMs = burstStartMs;
            }

            if (gamepad1.right_bumper) {

                long now = System.currentTimeMillis();
                boolean debouncePassed = (now - burstStartMs) >= FIRST_SHOT_DELAY_MS;
                boolean periodPassed   = (now - lastShotMs) >= SHOT_PERIOD_MS;

                if (debouncePassed && periodPassed && shooter.ready()) {
                    shooter.registerShotInBurst();
                    lastShotMs = now;
                }
            }

            if (gamepad1.rightBumperWasReleased()) {

            }

            // ===== TELEMETRY =====
            telemetry.addLine("=== TELEOP BLUE POLLEN SEARCH + SCORE ===");
            telemetry.addData("angle_adjust", ANGLE_ADJUST);
            telemetry.addData("useLimelightTurret", useLimelightTurret);
            telemetry.addData("limelightStarted", limelightStarted);
            telemetry.addData("useDynamicPower", useDynamicPower);
            telemetry.addData("PolenSearchDrive", polenActiv);
            if (polenActiv) {
                telemetry.addData("Polen vizibil", driveToPolen.bilaVizibila());
                telemetry.addData("Polen areComanda", driveToPolen.areComanda());
                telemetry.addData("Polen se cauta", driveToPolen.seCauta());
                telemetry.addData("Polen fwd/str/turn", "%.2f / %.2f / %.2f",
                        driveToPolen.getForward(), driveToPolen.getStrafe(), driveToPolen.getTurn());
                telemetry.addData("Polen stick override", stickMiscat);
                llBB.telemetrie();
            }

            telemetry.addData("Turret Emergency Fix", turela.isTurelaFixaActive());

            telemetry.addData("Dist(in)", "%.2f", calcRPM.getDistance());
            telemetry.addData("TargetRPM(raw)", "%.0f", calcRPM.getRPMRaw());
            telemetry.addData("TargetRPM(filt)", "%.0f", calcRPM.getRPM());

            telemetry.addData("LL started", limelightPoseStarted);
            if (limelightPose != null && limelightPose.getStatus() != null) {
                telemetry.addData("LL Pipeline", limelightPose.getStatus().getPipelineIndex());
            }

            //shooter.printTelemetry();
            telemetry.update();
        }
    }
}