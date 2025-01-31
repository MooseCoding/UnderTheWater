package org.firstinspires.ftc.teamcode.dairy.tuning

import android.graphics.Canvas
import android.util.Size
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.dairy.Sample
import org.firstinspires.ftc.teamcode.dairy.Color
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.max
import kotlin.math.min

@TeleOp
@Photon
@Disabled

class CameraTuning: OpMode() {

    private fun isRectangle(rotatedRect: RotatedRect): Boolean {
        val aspectRatio = min(rotatedRect.size.width, rotatedRect.size.height) / max(
            rotatedRect.size.width,
            rotatedRect.size.height
        )
        return aspectRatio > 0.8 && aspectRatio < 1.2 // roughly square
    }

    fun findRectanglesByColor(img: Mat?, lowerBound: Scalar?, upperBound: Scalar?, color: Color?) {
        // Convert the image to HSV
        val hsvImage = Mat()
        Imgproc.cvtColor(img, hsvImage, Imgproc.COLOR_BGR2HSV)

        // Threshold the HSV image to get only specific colors
        val mask = Mat()
        Core.inRange(hsvImage, lowerBound, upperBound, mask)

        // Find contours
        val contours = ArrayList<MatOfPoint>()
        val hierarchy = Mat()
        Imgproc.findContours(
            mask,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        // Iterate through contours and filter for rectangles
        for (contour in contours) {
            val contour2f = MatOfPoint2f(*contour.toArray())
            val rotatedRect = Imgproc.minAreaRect(contour2f)

            // Get the dimensions of the rectangle
            if (isRectangle(rotatedRect)) {
                // Draw the rectangle
                val rectPoints = arrayOfNulls<Point>(4)
                rotatedRect.points(rectPoints)


                if (rotatedRect.size.height > 100 && rotatedRect.size.width > 100) {
                    for (j in 0..3) {
                        Imgproc.line(
                            img,
                            rectPoints[j], rectPoints[(j + 1) % 4], Scalar(0.0, 255.0, 0.0), 2
                        )
                    }
                }

                if (samples.isEmpty()) {
                    samples.add(
                        Sample(
                            rotatedRect,
                            color!!
                        )
                    )
                }
            }
        }
    }

    private val lowerBlue = Scalar(100.0, 150.0, 50.0)
    private val upperBlue = Scalar(130.0, 255.0, 255.0)
    private val lowerRed1 = Scalar(0.0, 150.0, 50.0)
    private val upperRed1 = Scalar(10.0, 255.0, 255.0)
    private val lowerRed2 = Scalar(170.0, 150.0, 50.0)
    private val upperRed2 = Scalar(180.0, 255.0, 255.0)
    private val lowerYellow = Scalar(20.0, 150.0, 50.0)
    private val upperYellow = Scalar(30.0, 255.0, 255.0)
    var samples: java.util.ArrayList<Sample> = java.util.ArrayList()

    override fun init() {
        // Init the vision portal for the camera
        val mVP: VisionPortal
        val mVPB = VisionPortal.Builder()
        mVPB.setCamera(hardwareMap.get(WebcamName::class.java, "cam"))
            .setCameraResolution(Size(640, 480))
            .enableLiveView(true)
            .setAutoStopLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)


        mVPB.addProcessors(object : VisionProcessor {
            override fun init(width: Int, height: Int, calibration: CameraCalibration) {
            }

            override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
                findRectanglesByColor(frame, lowerYellow, upperYellow, Color.YELLOW)

                return null
            }

            override fun onDrawFrame(
                canvas: Canvas?,
                onscreenWidth: Int,
                onscreenHeight: Int,
                scaleBmpPxToCanvasPx: Float,
                scaleCanvasDensity: Float,
                userContext: Any?
            ) {
                TODO("Not yet implemented")
            }
        })

        mVP = mVPB.build()
    }

    override fun loop() {
        TODO("Not yet implemented")
    }
}