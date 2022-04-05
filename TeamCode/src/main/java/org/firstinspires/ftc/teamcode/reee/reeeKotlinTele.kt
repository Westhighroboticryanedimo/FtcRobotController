package org.firstinspires.ftc.teamcode.reee

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.Gyro

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "reee kotlin tele")

class reeeKotlinTele : OpMode() {
    var motor : DcMotor? = null
    var servo : Servo? = null
    var gyro : Gyro? = null
    var controller : Controller? = null
    val TURN_TICKS = 25.95

    var dir = false

    override fun init() {
        motor = hardwareMap.dcMotor.get("motor")
        servo = hardwareMap.servo.get("servo")
        gyro = Gyro(hardwareMap, false)
        controller = Controller(gamepad1)
        
        gyro!!.reset()
    }

    override fun loop() {
        if (controller!!.A()) {
            if (motor!!.getCurrentPosition() <= 0) {
                dir = true
            } else if (motor!!.getCurrentPosition() >= TURN_TICKS) {
                dir = false
            }

            if (dir) {
                motor!!.setPower(0.25)
            } else {
                motor!!.setPower(-0.25)
            }
        } else {
            motor!!.setPower(0.0)
        }

        servo!!.setPosition(gyro!!.getAngleDegrees()*(-180))

        telemetry.addData("ticks:", motor!!.getCurrentPosition())
        telemetry.update()
    }
}
