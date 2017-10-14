package madgwick.example.com.madgwick_filter;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends Activity {

    private SensorEventListener onRecieveAccListener;
    private SensorEventListener onRecieveGyroListener;
    private SensorEventListener onRecieveOrientationListener;
    private SensorEventListener onRecieveMagnetometrListener;
    private SensorManager mSensorManager;
    private Sensor mAcceleration;
    private Sensor mGyroscope;
    private Sensor mMagnetometr;
    private Sensor mOrientation;
    private MadgwickAHRS mMadgwickAHRS;
    private int counter = 0;


    private float ax, ay, az, gx, gy, gz, mx, my, mz;
    private float xy_angle, xz_angle, zy_angle;
    private TextView axView, ayView, azView,
            gxView, gyView, gzView;
    private TextView xyView, xzView, zyView, madZView,
            xyViewM, xzViewM, zyViewM, madZViewM,
            madYView, madXView, madWView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mMagnetometr = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mOrientation = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);

        axView = (TextView) findViewById(R.id.axView);
        ayView = (TextView) findViewById(R.id.ayView);
        azView = (TextView) findViewById(R.id.azView);
        gxView = (TextView) findViewById(R.id.gxView);
        gyView = (TextView) findViewById(R.id.gyView);
        gzView = (TextView) findViewById(R.id.gzView);
        xyView = (TextView) findViewById(R.id.xyView);
        xzView = (TextView) findViewById(R.id.xzView);
        zyView = (TextView) findViewById(R.id.zyView);
        xyViewM = (TextView) findViewById(R.id.xyViewM);
        xzViewM = (TextView) findViewById(R.id.xzViewM);
        zyViewM = (TextView) findViewById(R.id.zyViewM);
        madZView = (TextView) findViewById(R.id.madZView);
        madYView = (TextView) findViewById(R.id.madYView);
        madXView = (TextView) findViewById(R.id.madXView);
        madWView = (TextView) findViewById(R.id.madWView);


        onRecieveAccListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                    ax = event.values[0];
                    ay = event.values[1];
                    az = event.values[2];
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };
        onRecieveGyroListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                    gx = event.values[0];
                    gy = event.values[1];
                    gz = event.values[2];
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };
        onRecieveOrientationListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_ORIENTATION) {
                    xy_angle = event.values[0]; //Плоскость XY
                    xz_angle = event.values[1]; //Плоскость XZ
                    zy_angle = event.values[2]; //Плоскость ZY
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };

        onRecieveMagnetometrListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    mx = event.values[0]; //Плоскость XY
                    my = event.values[1]; //Плоскость XZ
                    mz = event.values[2]; //Плоскость ZY
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };

        Timer mTimer;
        GuiTimer guiTimer;
        mTimer = new Timer();
        guiTimer = new GuiTimer();
        mTimer.schedule(guiTimer, 0, 20); // 10Hz

        mMadgwickAHRS = new MadgwickAHRS(0.020f);
    }

    @Override
    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(onRecieveAccListener,
                mAcceleration,
                SensorManager.SENSOR_DELAY_GAME); // 50Hz, 20 ms delay
        mSensorManager.registerListener(onRecieveGyroListener,
                mGyroscope,
                SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(onRecieveOrientationListener,
                mOrientation,
                SensorManager.SENSOR_DELAY_GAME);
        mSensorManager.registerListener(onRecieveMagnetometrListener,
                mMagnetometr,
                SensorManager.SENSOR_DELAY_GAME);
    }

    @Override
    protected void onPause() {
        super.onPause();
        mSensorManager.unregisterListener(onRecieveAccListener);
        mSensorManager.unregisterListener(onRecieveGyroListener);
        mSensorManager.unregisterListener(onRecieveOrientationListener);
    }


    class GuiTimer extends TimerTask {

        @Override
        public void run() {
            runOnUiThread(new Runnable() {

                @Override
                public void run() {
                    mMadgwickAHRS.update(gx, gy, gz, ax, ay, az, mx, my, mz);

                    counter++;

                    if ( counter > 50 ) {
                        float[] quaternion = mMadgwickAHRS.getQuaternion();
                        float[] eulerAngles = mMadgwickAHRS.getEulerAngles();

                        madZView.setText(String.format ("%.3f", quaternion[3]));
                        madYView.setText(String.format ("%.3f", quaternion[2]));
                        madXView.setText(String.format ("%.3f", quaternion[1]));
                        madWView.setText(String.format ("%.3f", quaternion[0]));

                        axView.setText(String.format ("%.2f", ax));
                        ayView.setText(String.format ("%.2f", ay));
                        azView.setText(String.format ("%.2f", az));

                        gxView.setText(String.format ("%.2f", gx));
                        gyView.setText(String.format ("%.2f", gy));
                        gzView.setText(String.format ("%.2f", gz));

                        xyView.setText(String.format ("%.2f", xy_angle));
                        xzView.setText(String.format ("%.2f", xz_angle));
                        zyView.setText(String.format ("%.2f", zy_angle));

                        xyViewM.setText(String.format ("%.2f", eulerAngles[0]));
                        xzViewM.setText(String.format ("%.2f", eulerAngles[1]));
                        zyViewM.setText(String.format ("%.2f", eulerAngles[2]));

                        counter = 0;
                    }
                }
            });
        }
    }

}
