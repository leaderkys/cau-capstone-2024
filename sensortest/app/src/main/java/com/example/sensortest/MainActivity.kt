package com.example.sensortest

import android.annotation.SuppressLint
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.text.method.ScrollingMovementMethod
import android.widget.Button
import android.widget.TextView
import androidx.activity.ComponentActivity
import kotlin.math.pow
import kotlin.math.sqrt
import java.lang.System
import java.util.LinkedList
import java.util.Queue

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
                    return LinkedList<T>()
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

            var oneGTime: Long? = null
            var oneGTriggered = false

            val tRE = 350
            var tRESatisfied = false
            val tFE = 600
            var tFESatisfied = false


            /*vertical velocity related values*/
            val vertVelInterval = 120
            var vertVel = 0.0f
            val vertVelThres = -1.0
            var vertVelSatisfied = false

            private fun resetParams(){
                lftTriggered = false
                lftTime = null
                uftTime = null
                oneGTime = null
                oneGTriggered = false
                tRESatisfied = false
                tFESatisfied = false
                vertVelSatisfied = false
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


                val list = queue.get((vertVelInterval/ewma).toInt() + 1)
                if(list.isNotEmpty()){
                    vertVel = (list.sum()*ewma*.001).toFloat()
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
                            if(uftTime!! - lftTime!! < tFE){
                                accelSignificant.append("tFE satisfied\n")
                                tFESatisfied = true
                            }
                        }
                        if(oneGTime != null){
                            if(uftTime!! - oneGTime!! < tRE){
                                accelSignificant.append("tRE satisfied\n")
                                tRESatisfied = true
                            }
                        }
                        if(tFESatisfied && tRESatisfied && oneGTriggered && vertVelSatisfied){
                            accelSignificant.append("Fall Detected!!\n")
                        }
                        resetParams()
                    }
                }

                if(lftTriggered){
                    if(System.currentTimeMillis() - lftTime!! > lftTimeout){
                        resetParams()
                    }
                    else if(totalAccelG > 0.9 && totalAccelG < 1.01 && oneGTime == null){
                        accelSignificant.append(String.format("1-G: %.5f G\n",totalAccelG))
                        oneGTime = System.currentTimeMillis()
                        oneGTriggered = true
                    }
                }

                if(vertVel < vertVelThres){
                    accelSignificant.append(String.format("%.5f m/s\n",vertVel))
                    vertVelSatisfied = true
                }

                accelSensorData.text = String.format("Accel data:(dt=%d ms, EWMA=%.2f ms)\n" +
                        " x=%8.5f, y=%8.5f, z=%8.5f, \n total=%.5f g\n" +
                        " vertical velocity=%.5f m/s",
                                       interval, ewma,
                                       ax, ay, az, totalAccelG,
                                       vertVel)
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

