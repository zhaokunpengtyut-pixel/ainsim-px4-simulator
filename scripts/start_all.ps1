# Start PX4 and MAVLink relay
$px4 = Start-Process -WindowStyle Hidden -FilePath wsl.exe -ArgumentList "-d","Ubuntu-22.04","-u","hw","--","/home/hw/start_px4.sh" -PassThru
Write-Output "PX4 started PID=$($px4.Id)"
Start-Sleep -Seconds 20
# Check PX4 ports
wsl -d Ubuntu-22.04 -u hw ss -ulnp | findstr "185 145"
Write-Output "PX4 is running, now start QGC..."
Start-Process -FilePath "C:\Program Files\QGroundControl\bin\QGroundControl.exe"
