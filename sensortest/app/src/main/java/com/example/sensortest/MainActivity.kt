package com.example.sensortest

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.text.SpannableString
import android.text.method.ScrollingMovementMethod
import android.text.style.ForegroundColorSpan
import android.widget.Button
import android.widget.TextView
import androidx.activity.ComponentActivity
import kotlin.math.pow
import kotlin.math.sqrt
import java.lang.System
import java.util.LinkedList
import android.graphics.Color
import android.text.Spanned

class MainActivity : ComponentActivity() {

    @SuppressLint("SetTextI18n")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val accelSignificant : TextView = findViewById(R.id.accel_sensor_significant)
        val gyroSignificant : TextView = findViewById(R.id.gyro_sensor_significant)
        accelSignificant.text = ""
        accelSignificant.movementMethod = ScrollingMovementMethod()
        gyroSignificant.text = ""
        gyroSignificant.movementMethod = ScrollingMovementMethod()

        val clearBtn : Button = findViewById(R.id.clear_significant_history)
        clearBtn.setOnClickListener{
            accelSignificant.text = ""
            gyroSignificant.text = ""
        }

        val sensorInfo : TextView = findViewById(R.id.sensor_info)
        val accelSensorData : TextView = findViewById(R.id.accel_sensor_data)
        val gyroSensorData : TextView = findViewById(R.id.gyro_sensor_data)

        val sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        val accelList: List<Sensor> = sensorManager.getSensorList(Sensor.TYPE_ACCELEROMETER)
        val gyroList: List<Sensor> = sensorManager.getSensorList(Sensor.TYPE_GYROSCOPE)

        val sensorListBuilder = StringBuilder()

        sensorListBuilder.append("ACCELEROMETER: \n\n")
        for (sensors in accelList) {
            sensorListBuilder.append(sensors.toString() + "\n\n")
        }
        sensorListBuilder.append("GYRO: \n\n")
        for (sensors in gyroList) {
            sensorListBuilder.append(sensors.toString() + "\n\n")
        }
        sensorInfo.text = sensorListBuilder.toString()
        sensorInfo.movementMethod = ScrollingMovementMethod()

        val accel: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyro: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        class CircularBuffer<T>(private val maxSize: Int){
            private var head = 0
            private val linkedList = LinkedList<T>()

            fun add(item: T){
                if(linkedList.size >= maxSize){
                    linkedList.removeFirst()
                }
                linkedList.add(item)
            }

            fun get(len: Int): List<T>{
                if(len > linkedList.size){
                    return linkedList.subList(0, linkedList.size-1)
                }
                return linkedList.subList(linkedList.size-len, linkedList.size-1)
            }

            fun clear(){
                head = 0
                linkedList.clear()
            }
        }

        val linearAccelEventListener = object : SensorEventListener{
            val g = 9.81 //G = 9.81(m/s2)

            /*threshold values*/
            val aLFT = 0.65
            val aUFT = 2.8

            /*sensor polling interval measurements*/
            var startTime: Long = System.currentTimeMillis()
            var endTime: Long = 0
            var ewma = 0.0
            val weight = 0.125

            /*circular queue for vertical velocity integration*/
            val queueLimit = 200
            val queue = CircularBuffer<Double>(queueLimit)


            /*tFE and tRE measurements*/
            val lftTimeout = 1500
            var lftTriggered = false
            var lftTime: Long? = null

            var uftTime: Long? = null

            //var oneGTime: Long? = null
            //var oneGTriggered = false

            val tRE = 350
            val tREMin = 70
            var tRESatisfied = false
            val tFE = 600
            val tFEMin = 300
            var tFESatisfied = false


            /*vertical velocity related values*/
            //val vertVelInterval = 120
            val vertVelDeltaThres = 0.01
            var vertVel = 0.0f
            var vertVelDelta = 0.0f
            val vertVelMax = -1.0
            val vertVelMin = -0.3
            var vertVelSatisfied = false
            var vertVelOverspeed = false

            private fun resetParams(){
                lftTriggered = false
                lftTime = null
                uftTime = null
                //oneGTime = null
                //oneGTriggered = false
                tRESatisfied = false
                tFESatisfied = false
                vertVelSatisfied = false
                vertVelOverspeed = false
                queue.clear()
                accelSignificant.append("fall event reset\n")
            }

            @Suppress("SpellCheckingInspection")
            @SuppressLint("SetTextI18n", "DefaultLocale")
            override fun onSensorChanged(event: SensorEvent) {
                val ax = event.values[0]
                val ay = event.values[1]
                val az = event.values[2]
                val totalAccel = sqrt(ax.pow(2) + ay.pow(2) + az.pow(2))
                val totalAccelG = totalAccel/g
                queue.add(totalAccel-g)

                /*measure sensor sampling interval*/
                endTime = System.currentTimeMillis()
                val interval = endTime-startTime
                ewma = (1-weight)*ewma + weight*interval //EWMA is most reliable for interval value
                startTime = endTime


                /*
                val list = queue.get((vertVelInterval/ewma).toInt() + 1)
                if(list.isNotEmpty()){
                    vertVel = (list.sum()*ewma*.001).toFloat()
                }
                */

                vertVelDelta = ((totalAccel-g)*ewma*0.001).toFloat()
                //vertVelDelta = (( vertVelDelta * 1e2 ).toInt() / 1e3).toFloat()
                if(lftTriggered){
                    vertVel += vertVelDelta
                }else{
                    vertVel = 0.0f
                }

                if(totalAccelG < aLFT && !lftTriggered){
                    accelSignificant.append(String.format("LFT Triggered: %.5f G\n",totalAccelG))
                    lftTriggered = true
                    lftTime = System.currentTimeMillis()
                }

                if(totalAccelG > aUFT){
                    if(lftTriggered){
                        accelSignificant.append(String.format("UFT Triggered: %.5f G\n",totalAccelG))
                        uftTime = System.currentTimeMillis()
                        if(lftTime != null){
                            if( (uftTime!! - lftTime!! < tFE) && (uftTime!! - lftTime!! > tFEMin)){
                                accelSignificant.append("tFE satisfied\n")
                                tFESatisfied = true
                            }
                        }
                        /*
                        if(oneGTime != null){
                            if(uftTime!! - oneGTime!! < tRE){
                                accelSignificant.append("tRE satisfied\n")
                                tRESatisfied = true
                            }
                        }else{
                         */

                            val list = queue.get((tRE/ewma).toInt() + 1)
                            val start = ((tREMin)/ewma).toInt()
                            if(start < list.size){
                                var prev = list[start]
                                for(i in start until list.size){
                                    //accelSignificant.append(String.format("i=%.5f\n", i))
                                    if(list[i]*prev < 0){
                                        accelSignificant.append("tRE satisfied\n")
                                        tRESatisfied = true
                                        break
                                    }
                                    prev = list[i]
                                }
                            }
                        //}
                        if(tFESatisfied && tRESatisfied && vertVelSatisfied){
                            /*
                            val list = queue.get( ((uftTime!! - lftTime!!)/ewma).toInt()+1 )
                            accelSignificant.append("Printing accel data dump:\n")
                            for(i in list){
                                accelSignificant.append(String.format("%.5f\n", i))
                            }
                            */
                            val redColorString = SpannableString("Fall Detected!!\n")
                            redColorString.setSpan(ForegroundColorSpan(Color.RED), 0, redColorString.length, Spanned.SPAN_EXCLUSIVE_EXCLUSIVE)
                            accelSignificant.append(redColorString)
                        }
                        resetParams()
                    }
                }

                if(lftTriggered){
                    if(System.currentTimeMillis() - lftTime!! > lftTimeout){
                        resetParams()
                    }
                    /*
                    else if(totalAccelG > 0.9 && totalAccelG < 1.01 && oneGTime == null){
                        accelSignificant.append(String.format("1-G: %.5f G\n",totalAccelG))
                        oneGTime = System.currentTimeMillis()
                        oneGTriggered = true
                    }
                    */
                }

                if(lftTriggered && vertVel < vertVelMax){
                    accelSignificant.append(String.format("%.5f m/s OVERSPEED\n",vertVel))
                    vertVelOverspeed = true
                    vertVelSatisfied = false
                }
                if(lftTriggered && !vertVelOverspeed && vertVel < vertVelMin){
                    accelSignificant.append(String.format("%.5f m/s\n",vertVel))
                    vertVelSatisfied = true
                }

                accelSensorData.text = String.format("Accel data:(dt=%d ms, EWMA=%.2f ms)\n" +
                        " x=%8.5f, y=%8.5f, z=%8.5f, \n total=%.5f g\n" +
                        " vertical velocity: Δ%.5f m/s, total:%.5f m/s," ,
                                       interval, ewma,
                                       ax, ay, az, totalAccelG,
                                       vertVelDelta, vertVel)
            }

            @SuppressLint("SetTextI18n")
            override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
                accelSensorData.text = "Accuracy Changed!"
            }
        }

        val gyroEventListener = object : SensorEventListener{
            @Suppress("SpellCheckingInspection")
            val rThres = 20

            var startTime: Long = System.currentTimeMillis()
            var endTime: Long = 0
            var ewma = 0.0
            val weight = 0.125

            val queueLimit = 200
            val queue = CircularBuffer<Float>(queueLimit)

            @Suppress("SpellCheckingInspection")
            @SuppressLint("SetTextI18n", "DefaultLocale")
            override fun onSensorChanged(event: SensorEvent) {
                val rx = event.values[0]
                val ry = event.values[1]
                val rz = event.values[2]
                val totalRotation = sqrt(rx.pow(2) + ry.pow(2) + rz.pow(2))
                if(totalRotation > rThres){
                    gyroSignificant.append("$totalRotation\n")
                }
                endTime = System.currentTimeMillis()
                val interval = endTime-startTime
                ewma = (1-weight)*ewma + weight*interval
                startTime = endTime
                gyroSensorData.text = String.format("Gyro data:(dt=%d ms, EWMA=%.2f ms)\n" +
                        " x=%8.5f, y=%8.5f, z=%8.5f,\n total=%.5f",
                            interval, ewma,
                            rx, ry, rz, totalRotation)
            }

            @SuppressLint("SetTextI18n")
            override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
                gyroSensorData.text = "Accuracy Changed!"
            }
        }

        sensorManager.registerListener(linearAccelEventListener, accel, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(gyroEventListener, gyro, SensorManager.SENSOR_DELAY_GAME)

    }
}

