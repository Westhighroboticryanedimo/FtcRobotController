// package org.firstinspires.ftc.teamcode.reee.auto
// 
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
// import com.qualcomm.robotcore.util.ElapsedTime
// import com.qualcomm.robotcore.hardware.DistanceSensor
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
// 
// import org.firstinspires.ftc.teamcode.hardware.Gyro
// 
// import java.util.ArrayList
// 
// @Autonomous(name="reee kotlin auto")
// class auto : LinearOpMode() {
// 
//     override runOpMode() throws InterruptedException {
//         var ds : DistanceSensor = hardwareMap.distanceSensor.get("ds")
//         var time : ElapsedTime = ElapsedTime()
//         var values : ArrayList<Double> = ArrayList<Double>()
// 
//         waitForStart()
//         if (isStopRequested()) return
// 
//         while (time.seconds() < 10) {
//             values.add(ds.getDistance(DistanceUnit.INCH))
//         }
//         telemetry.addData("count:", values.size())
//         telemetry.update()
//         sleep(2000)
//     }
// }
// 
