package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import hardwareParts.absoluteEncoder;

@TeleOp(name = "Test Absolute Encoder")
public class testAbsoluteEncoder extends LinearOpMode {

    public DcMotor motor1 = null;

    private static final double kP = 0.008;
    // 0..1, mai mic = mai neted dar mai lent
    private static final double FILTER = 0.2;
    // dupa oprire, reporneste doar daca eroarea depaseste valoarea asta
    private static final double RESTART_ERROR = 8.0;
    private double filteredAngle = Double.NaN;
    private boolean holding = false;
    // daca motorul fuge de tinta in loc sa mearga spre ea, schimba in -1.0
    private static final double DIRECTION = 1.0;
    // sub eroarea asta (grade) motorul se opreste
    private static final double TOLERANCE = 4.0;
    // limita de putere ca sa nu fuga motorul
    private static final double MAX_POWER = 0.3;

    @Override
    public void runOpMode() {
        absoluteEncoder encoder = new absoluteEncoder(hardwareMap);
        motor1 = hardwareMap.get(DcMotor.class, "motorIntake");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double targetAngle = 22.0;

        // la init merge la 22 de grade si se opreste
        while (!isStarted() && !isStopRequested()) {
            encoder.update();
            driveToTarget(encoder, targetAngle);
        }

        waitForStart();

        while (opModeIsActive()) {
            encoder.update();

            // cross -> 180, square -> 270 (absolut), apoi se opreste la tinta
            if (gamepad1.cross) {
                targetAngle = 180.0;
            }
            else if (gamepad1.square) {
                targetAngle = 270.0;
            }

            driveToTarget(encoder, targetAngle);
        }
        motor1.setPower(0);
    }

    private void driveToTarget(absoluteEncoder encoder, double targetAngle) {
        double raw = normalizeAngle(encoder.getAngle());

        // filtru ca zgomotul senzorului (2-3 grade) sa nu miste motorul
        if (Double.isNaN(filteredAngle)) filteredAngle = raw;
        filteredAngle = normalizeAngle(filteredAngle + FILTER * shortestAngularError(filteredAngle, raw));
        double current = filteredAngle;
        double error = shortestAngularError(current, targetAngle);

        // histereza: dupa ce a ajuns la tinta, nu mai porneste pana nu iese din RESTART_ERROR
        if (Math.abs(error) <= TOLERANCE) holding = true;
        else if (Math.abs(error) > RESTART_ERROR) holding = false;

        double power = 0;
        if (!holding) {
            power = DIRECTION * kP * error;
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        }

        // cerc = motor oprit, ca sa poti roti de mana si citi voltajul
        if (gamepad1.circle) power = 0;
        motor1.setPower(power);

        telemetry.addData("Voltage", "%.3f V", encoder.getVoltage());
        telemetry.addData("Target",  "%.1f deg", targetAngle);
        telemetry.addData("Current", "%.1f deg", current);
        telemetry.addData("Error",   "%.1f deg", error);
        telemetry.addData("Power",   "%.3f",     power);
        telemetry.update();
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }

    private double shortestAngularError(double current, double target) {
        double error = target - current;
        while (error >  180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
