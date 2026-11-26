package teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestOdometrie", group = "Test")
public class TestOdometrie extends LinearOpMode {

    DcMotor odometrie;
    public static final double TICKS_PER_REV = 2000.0;
    public static final double WHEEL_DIAMETER_IN = 32.0 / 25.4; // ~1.8898 in
    public static final double INCHES_PER_TICK = (Math.PI * WHEEL_DIAMETER_IN) / TICKS_PER_REV;

    @Override
    public void runOpMode() {

        odometrie = hardwareMap.get(DcMotor.class, "odometrie");
        odometrie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometrie.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            double rawTicks = odometrie.getCurrentPosition(); // Read from DcMotorEx / DcMotor
            double distanceInInches = rawTicks * INCHES_PER_TICK;


            if (gamepad1.circleWasPressed()) {
                odometrie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }



            telemetry.addData("Connection", odometrie.getConnectionInfo());
            telemetry.addData("Pod Distance (in)", "%.2f", distanceInInches);
            telemetry.addData("Raw ticks: " , odometrie.getCurrentPosition());
            telemetry.update();


        }


    }


}
