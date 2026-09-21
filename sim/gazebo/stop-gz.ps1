# Stop the headless Gazebo server started by run-gz.ps1 (and avoid.py or fly.py if running).
Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -match 'stereo_avoid\.sdf|warehouse\.sdf|run-gz\.ps1|avoid\.py|fly\.py' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Write-Host "Gazebo stopped."
