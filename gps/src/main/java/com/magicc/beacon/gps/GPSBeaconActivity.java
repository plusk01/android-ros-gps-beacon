package com.magicc.beacon.gps;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.provider.Settings;
import android.util.Log;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class GPSBeaconActivity extends RosActivity
{
    private static final String NOTIFICATION_KEY = "GPSBeacon";
    protected static final String TAG = GPSBeaconActivity.class.getCanonicalName();
    private AndroidLocationNode locationNode;

    public GPSBeaconActivity() {
        super(NOTIFICATION_KEY, NOTIFICATION_KEY);
        // TODO Auto-generated constructor stub
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
    }

    /** Find out what location services are available. You may have to configure your location
     * services in the settings of your phone to allow high precision location measurements.
     */
    @Override
    protected void onResume() {
        super.onResume();

        // Acquire a reference to the system Location Manager
        LocationManager locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

        // Define a listener that responds to location updates
        LocationListener locationListener = new LocationListener() {
            private boolean gpsEnabled;

            public void onLocationChanged(Location location) {
                //take any location, unless the GPS is enabled...the only take GPS positions
                if (!gpsEnabled || location.getProvider().equals(LocationManager.GPS_PROVIDER)) {
                    // Called when a new location is found by the network location provider.
                    makeUseOfNewLocation(location);
                }
            }

            public void onStatusChanged(String provider, int status, Bundle extras) {}

            public void onProviderEnabled(String provider) {
                if (provider.equals(LocationManager.GPS_PROVIDER)) {
                    gpsEnabled = true;
                    Log.i(TAG, "Gps Enabled.  Suppressing network location");
                }
            }

            public void onProviderDisabled(String provider) {
                if (provider.equals(LocationManager.GPS_PROVIDER)) {
                    gpsEnabled = false;
                    Log.i(TAG, "Gps Disabled.  Using network location");
                }
            }
        };

        // Register the listener with the Location Manager to receive location updates
        locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0, 0, locationListener);
        locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);
    }

    /** send the location service message (hopefully GPS) to the ROS node to be published. */
    protected void makeUseOfNewLocation(Location location) {
        if (locationNode != null) {
            locationNode.updateLocation(location);
        }
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.
        String host = InetAddressFactory.newNonLoopback().getHostName();
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, getMasterUri());

        // Get a 'unique' name of the phone
        String deviceName = getPhoneName();

        // Execute the location node, named as the phone name
        locationNode = new AndroidLocationNode(deviceName);
        nodeMainExecutor.execute(locationNode, nodeConfiguration);
    }

    protected String getPhoneName() {
        // https://github.com/kakopappa/getdevcicename/blob/master/app/src/main/java/aruna/com/getdevcicename/MainActivity.java

        // Try to get the user defined phone name
        String deviceName = Settings.System.getString(getContentResolver(), "device_name");
        Log.v("MainActivity", "system device_name: " + deviceName);

        // If that didn't work, get the bluetooth device name
        if (deviceName == null || deviceName.isEmpty()) {
            BluetoothAdapter myDevice = BluetoothAdapter.getDefaultAdapter();
            deviceName = myDevice.getName();
            Log.v("MainActivity", "bluetooth device_name: " + deviceName);
        }

        // If that didn't work, just get the model name
        if (deviceName == null || deviceName.isEmpty()) {
            deviceName = android.os.Build.MODEL;
            Log.v("MainActivity", "Build.MODEL: " + deviceName);
        }

        // If all else fails, just call it "android." Name collisions will abound...
        if (deviceName == null || deviceName.isEmpty()) {
            deviceName = "android";
        }

        // return a lower-cased, no-spaced device name
        return deviceName.replaceAll(" ", "_").toLowerCase();
    }
}
