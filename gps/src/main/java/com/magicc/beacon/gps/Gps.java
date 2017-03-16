package com.magicc.beacon.gps;

import android.app.Activity;
import android.os.Bundle;

import org.ros.android.RosActivity;

public class Gps extends RosActivity
{
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
    }
}
