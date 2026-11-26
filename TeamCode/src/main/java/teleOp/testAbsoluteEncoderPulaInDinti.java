package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import hardwareParts.absoluteEncoder;

@TeleOp(name = "Test Absolute EncoderPulaInDinti")
public class testAbsoluteEncoderPulaInDinti extends LinearOpMode {

    public DcMotor motor1 = null;



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

            if (gamepad1.crossWasPressed()) {
                motor1.setPower(0.5);
                targetAngle=180;
            }

            else if (gamepad1.square) {
                motor1.setPower(0);
            }

            driveToTarget(encoder, targetAngle);
        }
    }

    private void driveToTarget(absoluteEncoder encoder, double targetAngle) {
        double raw = normalizeAngle(encoder.getAngle());

//        // filtru ca zgomotul senzorului (2-3 grade) sa nu miste motorul
//        if (Double.isNaN(filteredAngle)) filteredAngle = raw;
//        filteredAngle = normalizeAngle(filteredAngle + FILTER * shortestAngularError(filteredAngle, raw));
//        double current = filteredAngle;
//        double error = shortestAngularError(current, targetAngle);
//
//        // histereza: dupa ce a ajuns la tinta, nu mai porneste pana nu iese din RESTART_ERROR
//        if (Math.abs(error) <= TOLERANCE) holding = true;
//        else if (Math.abs(error) > RESTART_ERROR) holding = false;
//
//        double power = 0;
//        if (!holding) {
//            power = DIRECTION * kP * error;
//            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
//        }

        // cerc = motor oprit, ca sa poti roti de mana si citi voltajul
        //if (gamepad1.circle) motor1.setPower(0);

        telemetry.addData("Voltage", "%.3f V", encoder.getVoltage());
        telemetry.addData("Target",  "%.1f deg", targetAngle);
//        telemetry.addData("Current", "%.1f deg", current);
//        telemetry.addData("Error",   "%.1f deg", error);
//        telemetry.addData("Power",   "%.3f",     power);
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
