# Powershell Win32 boostrap waf
$url = "https://waf.io/waf-1.9.9"
$output = "$PSScriptRoot\waf.py"
$start_time = Get-Date
Invoke-WebRequest -Uri $url -OutFile $output
Write-Output "Time taken: $((Get-Date).Subtract($start_time).Seconds) second(s)"
