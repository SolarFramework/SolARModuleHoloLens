# Sample-HoloLens

Simple test loading the BuiltInSLAM component, connecting to the HoloLens device and streaming a single camera view and with its associated pose.

1. Build the [HoloLensForCV](https://github.com/SolarFramework/HoloLensForCV/tree/develop) project, and deploy the _StreamerVLC_ UWP app on the HoloLens device.
2. Run the _StreamerVLC_ app. 
3. Build & run ```Sample-HoloLens.exe``` (by default, the ```vlc_lf``` camera is selected)
4. All 4 vlc cameras are accessible, to select one particular camera, open a terminal and run:

    ```Sample-HoloLens.exe vlc_ll|vlc_lf|vlc_rf|vlc_rr```
