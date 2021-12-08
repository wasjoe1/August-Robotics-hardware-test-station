TO SETUP
- Run `./setup_script.sh augbooth` to put script into network manager on large PC.
- Run `./setup_script.sh augbooth i5` to put script into network manager on i5 large PC.
- Run `./setup_script.sh guiding` to put script into network manager on small PC

TO RUN
Now the remote access feature is ready to use.
3. Connect LAN cable to the correct LAN PORT.
    On BIG PC : Facing panel, Connect to LEFT port
    On SMALL PC : Facing Panel, Connect to LEFT port

4. Wait for ~30s for a notification on RemoteAccess_Test Dingtalk Group.
    The script to initiate a connection will not run again once there is 
    already an existing connection. (Only 1 connection at a time)
    If no notification is observed, perform step 5 and start again from step 3.

5. To end the session, simply unplug the LAN cable from the LAN PORT and wait
   for ~30s. This will automatically kill all existing ngrok processes on the
   PC.
   
TROUBLESHOOTING
Open a new terminal and run 'sudo tail -f /var/log/syslog'
Run 'nmcli device status' to view connected devices
Look out for the debug statements generated in the code.
