# SPDX-License-Identifier: Apache-2.0
param([string]$Port = "COM5", [int]$Baud = 1000000, [string]$Ids = "1", [int]$WaitMs = 30)
$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 50
$p.Open()
try {
  foreach ($idText in $Ids.Split(",")) {
    $id = [int]$idText
    $sum = ($id + 2 + 1) -band 0xFF
    $chk = (-bnot $sum) -band 0xFF
    [byte[]]$pkt = 0xFF, 0xFF, $id, 2, 1, $chk
    $p.DiscardInBuffer()
    $p.Write($pkt, 0, $pkt.Length)
    Start-Sleep -Milliseconds $WaitMs
    $n = $p.BytesToRead
    $buf = New-Object byte[] $n
    if ($n -gt 0) { [void]$p.Read($buf, 0, $n) }
    Write-Output ("baud {0} id {1}: {2} bytes: {3}" -f $Baud, $id, $n, (($buf | ForEach-Object { $_.ToString("X2") }) -join " "))
  }
} finally { $p.Close() }
