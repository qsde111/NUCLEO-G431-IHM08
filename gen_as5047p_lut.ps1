param(
    [string]$InputGlob = "Data/*.csv",
    [string]$OutputInc = "Core/Src/as5047P_lut_table.inc",
    [string]$AngleColumn = "angle_raw",
    [int]$BinShift = 4,
    [int]$SkipSamples = 0
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$mask = 0x3FFF
$fullScale = 16384
$bins = $fullScale -shr $BinShift
if (($bins -shl $BinShift) -ne $fullScale) { throw "BinShift=$BinShift does not divide 16384 evenly." }
if ($bins -le 0) { throw "Invalid bins=$bins" }

$hist = New-Object 'uint64[]' $bins
$total = [uint64]0

$files = Get-ChildItem -Path $InputGlob -ErrorAction Stop | Sort-Object FullName
if ($files.Count -eq 0) { throw "No files match: $InputGlob" }

foreach ($f in $files) {
    Write-Host ("Reading {0}" -f $f.FullName)
    $rows = Import-Csv $f.FullName
    if ($rows.Count -le $SkipSamples) { continue }

    for ($i = $SkipSamples; $i -lt $rows.Count; $i++) {
        $raw = [int][math]::Round([double]$rows[$i].$AngleColumn)
        $raw = $raw -band $mask
        $idx = $raw -shr $BinShift
        $hist[$idx]++
        $total++
    }
}

if ($total -eq 0) { throw "No samples collected (check SkipSamples/AngleColumn)." }

$lut = New-Object 'uint16[]' ($bins + 1)
$lut[0] = 0
$sum = [uint64]0
for ($i = 1; $i -le $bins; $i++) {
    $sum += $hist[$i - 1]
    $val = [int][math]::Round(($sum * $fullScale * 1.0) / $total)
    if ($val -lt $lut[$i - 1]) { $val = $lut[$i - 1] }
    if ($val -gt $fullScale) { $val = $fullScale }
    $lut[$i] = [uint16]$val
}
$lut[$bins] = $fullScale

# format: 16 entries per line, trailing comma is OK in C initializers
$lines = New-Object 'System.Collections.Generic.List[string]'
for ($i = 0; $i -le $bins; $i += 16) {
    $end = [math]::Min($i + 15, $bins)
    $chunk = @()
    for ($j = $i; $j -le $end; $j++) { $chunk += $lut[$j] }
    $lines.Add("  " + ($chunk -join ", ") + ",")
}

$outDir = Split-Path -Parent $OutputInc
if ($outDir -and -not (Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }
$lines | Set-Content -Encoding ascii -Path $OutputInc

Write-Host ("Wrote {0} entries to {1}" -f ($bins + 1), $OutputInc)
Write-Host ("Total samples used: {0}" -f $total)

