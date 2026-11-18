# SPDX-License-Identifier: Apache-2.0
param([string]$Port = "COM5", [int]$Baud = 1000000, [int]$Id = 1, [int]$From = 0, [int]$To = 69, [int]$Repeat = 1, [int]$GapMs = 0)
$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.Open()
function ReadBlock([int]$addr, [int]$len) {
  $sum = ($Id + 4 + 2 + $addr + $len) -band 0xFF
  [byte[]]$pkt = 0xFF, 0xFF, $Id, 4, 2, $addr, $len, ((-bnot $sum) -band 0xFF)
  $p.DiscardInBuffer()
  $p.Write($pkt, 0, $pkt.Length)
  $want = 6 + $len
  $deadline = [DateTime]::Now.AddMilliseconds(60)
  while ($p.BytesToRead -lt $want -and [DateTime]::Now -lt $deadline) { Start-Sleep -Milliseconds 1 }
  $n = $p.BytesToRead
  $buf = New-Object byte[] $n
  if ($n -gt 0) { [void]$p.Read($buf, 0, $n) }
  return ,$buf
}
try {
  for ($r = 0; $r -lt $Repeat; $r++) {
    $a = $From
    while ($a -le $To) {
      $len = [Math]::Min(16, $To - $a + 1)
      $buf = ReadBlock $a $len
      Write-Output ("{0} addr {1} len {2}: {3}" -f ([DateTime]::Now.ToString("HH:mm:ss.fff")), $a, $len, (($buf | ForEach-Object { $_.ToString() }) -join " "))
      $a += $len
    }
    if ($GapMs -gt 0) { Start-Sleep -Milliseconds $GapMs }
  }
} finally { $p.Close() }
