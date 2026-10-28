<#
  From a fresh Windows PC to a checked install, in one step: a virtual environment (.venv), the pinned packages, the vendor
  arm SDK wheel if you have it, and the doctor (a read-only check of this PC and the cell).

    .\install.ps1                                    the real cell
    .\install.ps1 -SdkWheel C:\path\to\the-vendor-sdk.whl
    .\install.ps1 -Sim                               a PC with no cell: the simulator only (no pythonnet, no SDK)

  Then:   .\.venv\Scripts\Activate.ps1   and   python cli.py setup
#>
param(
    [string]$SdkWheel = "",
    [switch]$Sim,
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot

function Step($text) { Write-Host "`n== $text" }
function Fail($text) { Write-Host "`nFAILED: $text" -ForegroundColor Red; exit 1 }

Step "Python"
try {
    $version = & $Python -c "import sys; print('%d.%d.%d' % sys.version_info[:3]); sys.exit(0 if sys.version_info >= (3, 10) else 1)"
} catch {
    Fail "no usable Python found. Install Python 3.10 or newer from python.org (tick 'Add python.exe to PATH'), or pass -Python <path to python.exe>."
}
if ($LASTEXITCODE -ne 0) { Fail "Python $version is too old: 3.10 or newer is needed (python.org)." }
Write-Host "Python $version"

Step "Virtual environment (.venv)"
if (-not (Test-Path ".venv\Scripts\python.exe")) {
    & $Python -m venv .venv
    if ($LASTEXITCODE -ne 0) { Fail "could not create .venv" }
}
$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

Step "Packages (pinned in requirements.txt)"
& $venvPython -m pip install --upgrade pip --quiet
$requirements = "requirements.txt"
if ($Sim) {
    $requirements = Join-Path $env:TEMP "requirements-sim.txt"
    Get-Content "requirements.txt" | Where-Object { $_ -notmatch "^\s*pythonnet" } | Set-Content -Path $requirements -Encoding ASCII
}
& $venvPython -m pip install -r $requirements
if ($LASTEXITCODE -ne 0) { Fail "pip could not install the packages. Check the network, then run install.ps1 again." }

if ($SdkWheel -ne "") {
    Step "Vendor arm SDK"
    if (-not (Test-Path $SdkWheel)) { Fail "the wheel $SdkWheel does not exist." }
    & $venvPython -m pip install $SdkWheel
    if ($LASTEXITCODE -ne 0) { Fail "pip could not install $SdkWheel." }
} elseif (-not $Sim) {
    Write-Host "`nNo -SdkWheel given: the real arm needs the vendor wheel (and the .NET 8 runtime). The doctor says if it is missing."
}

Step "Doctor (reads only, moves nothing)"
if ($Sim) { & $venvPython tools\doctor.py --sim } else { & $venvPython tools\doctor.py }
$doctorExit = $LASTEXITCODE

$next = "python cli.py setup"
if ($Sim) { $next = "$next --sim" }
Write-Host "`nNext:  .\.venv\Scripts\Activate.ps1   then   $next"
exit $doctorExit
