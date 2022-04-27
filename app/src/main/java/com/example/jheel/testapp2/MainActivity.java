package com.example.jheel.testapp2;

import android.Manifest;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.location.Location;
import android.location.LocationManager;
import android.provider.Settings;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.DisplayMetrics;
import android.util.Log;
import android.util.SizeF;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class MainActivity extends AppCompatActivity implements SensorEventListener, CameraBridgeViewBase.CvCameraViewListener2, GoogleApiClient.ConnectionCallbacks, GoogleApiClient.OnConnectionFailedListener, com.google.android.gms.location.LocationListener {

    float mag, d_x, d_y;
    public float[] loc_p;
    float[] R_p;
    float[] r_r = new float[]{0, 0, 0};
    //Orientation sensor objects
    SensorManager mSensorManager;
    Sensor accelerometer;
    Sensor magnetometer;
    public float yaw, pitch, roll;


    //fov object
    static final int REQUEST_CAMERA = 1;

    //Location object
    public float[] Loc;
    private GoogleApiClient mGoogleApiClient;
    private Location mLocation;
    private LocationManager mLocationManager;
    private LocationRequest mLocationRequest;
    private com.google.android.gms.location.LocationListener listener;
    private long UPDATE_INTERVAL = 2 * 1000;  /* 10 secs */
    private long FASTEST_INTERVAL = 2000; /* 2 sec */
    private LocationManager locationManager;


    // Used for logging success or failure messages
    private static final String TAG = "OCVSample::Activity";

    // Loads camera view of OpenCV for us to use. This lets us see using OpenCV
    private CameraBridgeViewBase mOpenCvCameraView;

    // These variables are used (at the moment) to fix camera orientation from 270degree to 0degree
    Mat mRgba;
    Mat mRgbaF;
    Mat mRgbaT;
    int[] layoutdim;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[] {android.Manifest.permission.CAMERA}, 0);
        }
        mOpenCvCameraView = (JavaCameraView) findViewById(R.id.show_camera_activity_java_surface_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        mGoogleApiClient = new GoogleApiClient.Builder(this)
                .addConnectionCallbacks(this)
                .addOnConnectionFailedListener(this)
                .addApi(LocationServices.API)
                .build();

        mLocationManager = (LocationManager)this.getSystemService(Context.LOCATION_SERVICE);
        checkLocation(); //check whether location service is enable or not in your  phone
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        calculateFOV(manager);

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        //loc = {altitude, latitude, longitude}
        Loc = new float[]{0, (float) Math.toRadians(22.354378), (float) Math.toRadians(73.174611)};
        loc_p = new float[]{(float) 0, (float) Math.toRadians(23.510343), (float) Math.toRadians(72.119298)};
    }

    float horizonalAngle;
    float verticalAngle;

    private void calculateFOV(CameraManager cManager) {
        try {
            for (final String cameraId : cManager.getCameraIdList()) {
                CameraCharacteristics characteristics = cManager.getCameraCharacteristics(cameraId);
                int cOrientation = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (cOrientation == CameraCharacteristics.LENS_FACING_BACK) {
                    float[] maxFocus = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
                    SizeF size = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
                    float w = size.getWidth();
                    float h = size.getHeight();
                    horizonalAngle = (float) (2*Math.atan(w/(maxFocus[0]*2)));
                    verticalAngle = (float) (2*Math.atan(h/(maxFocus[0]*2)));
                }
            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }
    public float getFOV(){
        return verticalAngle;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode) {
            case REQUEST_CAMERA:
                for (int i = 0; i < permissions.length; i++) {
                    String permission = permissions[i];
                    int grantResult = grantResults[i];
                    if (permission.equals(Manifest.permission.CAMERA)) {
                        if(grantResult == PackageManager.PERMISSION_GRANTED) {
                            Log.d(TAG, "mPreview set");
                        } else {
                            Toast.makeText(this, "Should have camera permission to run", Toast.LENGTH_LONG).show();
                            finish();
                        }
                    }
                }
                break;
        }
    }


    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {

        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mRgbaF = new Mat(height, width, CvType.CV_8UC4);
        mRgbaT = new Mat(width, width, CvType.CV_8UC4);
        layoutdim = new int[] {width, height};
    }

    public int[] getLayoutdimensions() {
        return layoutdim;
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        // TODO Auto-generated method stub
        mRgba = inputFrame.rgba();
        // Rotate mRgba 90 degrees
        Core.transpose(mRgba, mRgbaT);
        Imgproc.resize(mRgbaT, mRgbaF, mRgbaF.size(), 0,0, 0);
        Core.flip(mRgbaF, mRgba, 1 );
        int[] npoints;
        npoints = get_relative_location();
        Imgproc.putText(mRgba, Integer.toString(npoints[0])+", "+ Integer.toString(npoints[1]), new Point(20, 50), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Imgproc.putText(mRgba, Float.toString(getScreenDimension())+", "+Float.toString(verticalAngle
        ), new Point(20, 100), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Imgproc.putText(mRgba, Float.toString((float) Math.toDegrees(yaw))+", "+ Float.toString((float) Math.toDegrees(pitch))+", "+ Float.toString((float) Math.toDegrees(roll)), new Point(20, 150), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Imgproc.putText(mRgba, Float.toString(r_r[0])+", "+ Float.toString(r_r[1])+", "+ Float.toString(r_r[2]), new Point(20, 200), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Imgproc.putText(mRgba, Float.toString(mag)+", "+ Float.toString(d_x)+", "+ Float.toString(d_x), new Point(20, 250), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        Imgproc.putText(mRgba, Float.toString(Loc[0])+", "+ Float.toString(Loc[1])+", "+ Float.toString(Loc[2]), new Point(20, 650), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);

        Imgproc.rectangle(mRgba, new Point(npoints[0], npoints[1]), new Point(npoints[0]+8, npoints[1]+8), new Scalar(0, 255, 0), 4);
        return mRgba; // This function must return
    }

    public float getScreenDimension() {
        DisplayMetrics metrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(metrics);
        return (float) metrics.densityDpi;
    }



    @Override
    public void onConnected(@Nullable Bundle bundle) {
        if (ActivityCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }

        startLocationUpdates();

        mLocation = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);

        if(mLocation == null){
            startLocationUpdates();
        }
        if (mLocation != null) {

            // mLatitudeTextView.setText(String.valueOf(mLocation.getLatitude()));
            //mLongitudeTextView.setText(String.valueOf(mLocation.getLongitude()));
        } else {
            Toast.makeText(this, "Location not Detected", Toast.LENGTH_SHORT).show();
        }

    }

    @Override
    public void onConnectionSuspended(int i) {
        Log.i(TAG, "Connection Suspended");
        mGoogleApiClient.connect();
    }

    @Override
    public void onConnectionFailed(ConnectionResult connectionResult) {
        Log.i(TAG, "Connection failed. Error: " + connectionResult.getErrorCode());
    }

    @Override
    protected void onStart() {
        super.onStart();
        if (mGoogleApiClient != null) {
            mGoogleApiClient.connect();
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        if (mGoogleApiClient.isConnected()) {
            mGoogleApiClient.disconnect();
        }
    }

    protected void startLocationUpdates() {
        // Create the location request
        mLocationRequest = LocationRequest.create()
                .setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY)
                .setInterval(UPDATE_INTERVAL)
                .setFastestInterval(FASTEST_INTERVAL);
        // Request location updates
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, this);
        Log.d("reque", "--->>>>");
    }

    @Override
    public void onLocationChanged(Location location) {
        Loc[0] = (float) location.getAltitude();
        Loc[1] = (float) Math.toRadians(location.getLatitude());
        Loc[2] = (float) Math.toRadians(location.getLongitude());
    }

    private boolean checkLocation() {
        if(!isLocationEnabled())
            showAlert();
        return isLocationEnabled();
    }

    private void showAlert() {
        final AlertDialog.Builder dialog = new AlertDialog.Builder(this);
        dialog.setTitle("Enable Location")
                .setMessage("Your Locations Settings is set to 'Off'.\nPlease Enable Location to " +
                        "use this app")
                .setPositiveButton("Location Settings", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface paramDialogInterface, int paramInt) {

                        Intent myIntent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                        startActivity(myIntent);
                    }
                })
                .setNegativeButton("Cancel", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface paramDialogInterface, int paramInt) {

                    }
                });
        dialog.show();
    }

    private boolean isLocationEnabled() {
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        return locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
                locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER);
    }

    public float[] getLocation() {
        return Loc;
    }

    float[] mGravity;
    float[] mGeomagnetic;


    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            mGravity = event.values;
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            mGeomagnetic = event.values;
        if (mGravity != null && mGeomagnetic != null) {
            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            float[] outGravity = new float[9];
//            SensorManager.remapCoordinateSystem(R, SensorManager.AXIS_X, SensorManager.AXIS_Z, outGravity);
            if (success) {
                float[] values = new float[3];
                SensorManager.getOrientation(R, values);
                yaw = values[0];
                pitch = values[1];
                roll = values[2];
            }
            mGravity = null;
            mGeomagnetic = null;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }


    public float[] cal_cross(float[] u, float[] v){

        float u1, u2, u3, v1, v2, v3;
        u1=u[0];
        u2=u[1];
        u3=u[2];
        v1=v[0];
        v2=v[1];
        v3=v[2];
        float uvi, uvj, uvk;
        uvi = u2 * v3 - v2 * u3;
        uvj = v1 * u3 - u1 * v3;
        uvk = u1 * v2 - v1 * u2;
        float[] uv;
        uv= new float[]{uvi, uvj, uvk};
        return uv;
    }

    private float cal_dot(float[] u, float[] v) {
        float uv;
        uv=0;
        for (int i=0; i<3; i++) {
            uv += u[i]*v[i];
        }
        return uv;
    }

    public int[] get_relative_location() {
        float l;
        float[] loc_m;
        loc_m = getLocation();
        float r;
        r=6400000;
        float[] R_p_;
        float[] R_m_;
        int nlayoutdim[] = getLayoutdimensions();
        float h = (float) ((float) nlayoutdim[1]*0.0254/getScreenDimension());
        float w = (float) ((float) nlayoutdim[0]*0.0254/getScreenDimension());
        //loc = {altitude, latitude, longitude}
        R_p_= new float[]{(float) ((r+loc_p[0])*Math.sin(loc_p[2])*Math.cos(loc_p[1])), (float) ((r+loc_p[0])*Math.sin(loc_p[2])*Math.sin(loc_p[1])), (float) ((r+loc_p[0])*Math.cos(loc_p[2]))};
        R_m_= new float[]{(float) ((r+loc_m[0])*Math.sin(loc_m[2])*Math.cos(loc_m[1])), (float) ((r+loc_m[0])*Math.sin(loc_m[2])*Math.sin(loc_m[1])), (float) ((r+loc_m[0])*Math.cos(loc_m[2]))};
        R_p = new float[]{-R_p_[0]+R_m_[0], -R_p_[1]+R_m_[1], -R_p_[2]+R_m_[2]};
        R_p = cal_unit(R_p);
        float[] v_1, v_2, v_3;
        v_1=new float[]{(float) (Math.cos(yaw)*Math.cos(pitch)), (float) (Math.sin(yaw)*Math.cos(pitch)), (float) Math.sin(pitch)};
        v_2=new float[]{(float) (-Math.cos(yaw)*Math.sin(pitch)*Math.sin(roll)-Math.sin(yaw)*Math.cos(roll)), (float) (-Math.sin(yaw)*Math.sin(pitch)*Math.sin(roll)+Math.cos(yaw)*Math.cos(roll)), (float) (Math.cos(pitch)*Math.sin(roll))};
        v_3 = cal_cross(v_1, v_2);
        v_3 = cal_unit(v_3);
        l = (float) (h/(2*Math.tan(getFOV()/2)));
        r_r[0] = cal_dot(R_p, v_1);
        r_r[1] = cal_dot(R_p, v_2);
        r_r[2] = cal_dot(R_p, v_3);
        r_r = cal_unit(r_r);

        float[] display = new float[]{0, 0};
        if(r_r[2] != 0){
            mag=Math.abs(l/r_r[2]);
            d_x=r_r[0]*mag;
            d_y=r_r[1]*mag;
            d_x= (float) (d_x*getScreenDimension()/0.0254);
            d_y= (float) (d_y*getScreenDimension()/0.0254);

            display[1] = nlayoutdim[1]/2 - d_x;
            display[0] = nlayoutdim[0]/2 + d_y;

            if ((Math.abs(d_x) > nlayoutdim[1]/2) || (Math.abs(d_y) > nlayoutdim[0]/2)){
                float ele = d_y/d_x;
                if (Math.abs(ele)>(nlayoutdim[1]/nlayoutdim[0])) {
                    display[1] = nlayoutdim[1]/2 - (1/ele)*(Math.abs(d_x)/d_x)*nlayoutdim[1]/2;
                    display[0] = nlayoutdim[0]/2 + (Math.abs(d_y)/d_y)*nlayoutdim[0]/2;
                }
                if (Math.abs(ele)<(nlayoutdim[1]/nlayoutdim[0])) {
                    display[1] = nlayoutdim[1]/2 - (Math.abs(d_x)/d_x)*nlayoutdim[1]/2;
                    display[0] = nlayoutdim[0]/2 + ele*(d_y/Math.abs(d_y))*nlayoutdim[0]/2;
                }
            }
        }
//        if(r_r[2] > 0){
//            mag=l/r_r[2];
//            d_x=r_r[0]*mag;
//            d_y=r_r[1]*mag;
//            d_x= (float) (d_x*getScreenDimension()/0.0254);
//            d_y= (float) (d_y*getScreenDimension()/0.0254);
//
//            display[0] = d_x + w/2;
//            display[1] = h/2 - d_y;
//
//            if ((Math.abs(d_x) > w/2) || (Math.abs(d_y) > h/2)){
//                float ele = d_y/d_x;
//                if (Math.abs(ele)>(h/w)) {
//                    display[0] = (1/Math.abs(ele))*(Math.abs(d_x)/d_x)*w/2;
//                    display[1] = (Math.abs(d_y)/d_y)*h/2;
//                }
//                if (Math.abs(ele)<(h/w)) {
//                    display[0] = (Math.abs(d_x)/d_x)*w/2;
//                    display[1] = Math.abs(ele)*(d_y/Math.abs(d_y))*h/2;
//                }
//            }
//        }
        int[] int_display = new int[2];
        int_display[0] = (int) (display[0]);
        int_display[1] = (int) (display[1]);
        return int_display;
    }
    private float[] cal_unit(float[] v) {
        return new float[]{(float) (v[0]/Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])), (float) (v[1]/Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])), (float) (v[2]/Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]))};
    }
}


