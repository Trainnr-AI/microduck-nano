# SPDX-License-Identifier: Apache-2.0
# Bench test of ONE SCS0009 through the Waveshare adapter's USB port: centre it, then sweep it at rising speeds.
# RAM registers only (torque enable 40, goal position / time / speed 42..47); nothing is written to the servo's
# permanent memory. Torque is switched off at the end and on any abort.
param([string]$Port = "COM5", [int]$Id = 1, [string]$Speeds = "150,300,450,600,750,900,1050,1200,1350,1500",
      [int]$Centre = 512, [int]$Swing = 205, [switch]$AlsoSpeedZero)
$p = New-Object System.IO.Ports.SerialPort $Port, 1000000, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.Open()
$sw = [System.Diagnostics.Stopwatch]::StartNew()
function Exchange([byte[]]$body, [int]$want) {
  $sum = 0; foreach ($b in $body) { $sum += $b }
  [byte[]]$pkt = @(0xFF, 0xFF) + $body + @((-bnot $sum) -band 0xFF)
  $p.DiscardInBuffer(); $p.Write($pkt, 0, $pkt.Length)
  $deadline = $sw.ElapsedMilliseconds + 40
  while ($p.BytesToRead -lt $want -and $sw.ElapsedMilliseconds -lt $deadline) { }
  $n = $p.BytesToRead; $buf = New-Object byte[] $n
  if ($n -gt 0) { [void]$p.Read($buf, 0, $n) }
  return ,$buf
}
function WriteReg([int]$addr, [byte[]]$data) {
  [byte[]]$body = @($Id, ($data.Length + 3), 3, $addr) + $data
  $r = Exchange $body 6
  if ($r.Length -ne 6) { return -1 }
  return [int]$r[4]
}
function ReadState() {
  $r = Exchange ([byte[]]@($Id, 4, 2, 56, 8)) 14
  if ($r.Length -ne 14 -or $r[0] -ne 255 -or $r[2] -ne $Id) { return $null }
  return @{ err = [int]$r[4]; pos = $r[5] * 256 + $r[6]; spd = $r[7] * 256 + $r[8]; load = $r[9] * 256 + $r[10]; volt = [int]$r[11]; temp = [int]$r[12] }
}
function Goal([int]$pos, [int]$speed) {
  return WriteReg 42 ([byte[]]@(($pos -shr 8), ($pos -band 255), 0, 0, ($speed -shr 8), ($speed -band 255)))
}
$script:abort = ""
$script:stall = 0
function Watch([string]$label, [int]$goal, [int]$speed, [int]$maxMs) {
  $start = $sw.ElapsedMilliseconds; $still = 0
  while ($sw.ElapsedMilliseconds - $start -lt $maxMs) {
    $s = ReadState
    if ($null -eq $s) { Write-Output ("{0},{1},{2},{3},noreply" -f $sw.ElapsedMilliseconds, $label, $goal, $speed); continue }
    Write-Output ("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9}" -f $sw.ElapsedMilliseconds, $label, $goal, $speed, $s.pos, $s.spd, $s.load, $s.volt, $s.temp, $s.err)
    if (($s.err -band 0x24) -ne 0) { $script:abort = "servo reports overload or overheat (error byte $($s.err))"; return }
    if ($s.temp -ge 55) { $script:abort = "temperature $($s.temp) C"; return }
    if (($s.load -band 0x3FF) -ge 900) { $script:stall++ } else { $script:stall = 0 }
    if ($script:stall -ge 450) { $script:abort = "load at the limit for 450 readings, about half a second (stalled)"; return }
    if ([Math]::Abs($s.pos - $goal) -le 3 -and ($s.spd -band 0x7FFF) -eq 0) { $still++ } else { $still = 0 }
    if ($still -ge 12) { return }
  }
}
try {
  $s = ReadState
  if ($null -eq $s) { throw "the servo does not answer" }
  Write-Output ("# start: position {0}, {1} V x10, {2} C, error byte {3}" -f $s.pos, $s.volt, $s.temp, $s.err)
  # Torque on without a jump: the goal is where the servo already is.
  [void](Goal $s.pos 150)
  $e = WriteReg 40 ([byte[]]@(1))
  Write-Output ("# torque on: reply error byte {0}" -f $e)
  [void](Goal $Centre 150)
  Watch "centre" $Centre 150 8000
  $list = @($Speeds.Split(",") | ForEach-Object { [int]$_ })
  if ($AlsoSpeedZero) { $list += 0 }
  foreach ($speed in $list) {
    if ($script:abort) { break }
    $budget = if ($speed -gt 0) { [int](2 * $Swing * 1000 / $speed) + 1500 } else { 2500 }
    foreach ($goal in @(($Centre - $Swing), ($Centre + $Swing), $Centre)) {
      if ($script:abort) { break }
      [void](Goal $goal $speed)
      Watch ("speed" + $speed) $goal $speed $budget
    }
  }
  if ($script:abort) { Write-Output ("# ABORTED: " + $script:abort) }
} finally {
  $e = WriteReg 40 ([byte[]]@(0))
  Write-Output ("# torque off: reply error byte {0}" -f $e)
  $s = ReadState
  if ($null -ne $s) { Write-Output ("# end: position {0}, {1} V x10, {2} C, error byte {3}" -f $s.pos, $s.volt, $s.temp, $s.err) }
  $p.Close()
}
