param(
    [string]$BaseUrl = "http://p4-buoy.local",
    [string]$FallbackBaseUrl = "http://169.254.100.2",
    [int]$Limit = 5,
    [string]$OutDir = "artifacts/ethernet_downloads",
    [string]$InterfaceAddress = "",
    [switch]$SkipExportMode,
    [switch]$StopRecordingFirst
)

$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$script:ActiveBaseUrl = ""
$script:ActiveInterfaceAddress = ""

function Normalize-BaseUrl {
    param([string]$Url)
    return $Url.Trim().TrimEnd("/")
}

function Get-AutoInterfaceAddress {
    param([string]$RemoteHost)

    try {
        $route = Find-NetRoute -RemoteIPAddress $RemoteHost -ErrorAction Stop | Select-Object -First 1
        if ($route) {
            $addr = Get-NetIPAddress -AddressFamily IPv4 -InterfaceIndex $route.InterfaceIndex -ErrorAction Stop |
                Where-Object { $_.AddressState -eq "Preferred" -and $_.IPAddress -like "169.254.*" } |
                Select-Object -First 1
            if ($addr) {
                return [string]$addr.IPAddress
            }
        }
    } catch {
        # Fall back to scanning local APIPA addresses below.
    }

    $addr = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object {
            $_.AddressState -eq "Preferred" -and
            $_.IPAddress -like "169.254.*" -and
            $_.InterfaceAlias -notmatch "Bluetooth|Wi-Fi Direct"
        } |
        Select-Object -First 1

    if ($addr) {
        return [string]$addr.IPAddress
    }

    return ""
}

function Get-InterfaceForBaseUrl {
    param([string]$Base)

    if ($InterfaceAddress) {
        return $InterfaceAddress
    }

    $uri = [Uri]$Base
    if ($uri.Host -like "169.254.*") {
        return Get-AutoInterfaceAddress -RemoteHost $uri.Host
    }

    return ""
}

function Set-ActiveBaseUrl {
    param([string]$Base)

    $script:ActiveBaseUrl = Normalize-BaseUrl $Base
    $script:ActiveInterfaceAddress = Get-InterfaceForBaseUrl -Base $script:ActiveBaseUrl
}

function Invoke-ActiveCurlText {
    param(
        [string]$Url,
        [string]$Method = "GET",
        [string]$Body = "",
        [int]$TimeoutSec = 30
    )

    $args = @(
        "--silent", "--show-error", "--fail", "--http1.1",
        "--connect-timeout", "10",
        "--max-time", [string]$TimeoutSec
    )
    if ($script:ActiveInterfaceAddress) {
        $args += @("--interface", $script:ActiveInterfaceAddress)
    }
    if ($Method -ne "GET") {
        $args += @("--request", $Method)
    }
    if ($Body) {
        $args += @(
            "--header", "Content-Type: application/x-www-form-urlencoded",
            "--data", $Body
        )
    }
    $args += $Url

    $text = & curl.exe @args
    if ($LASTEXITCODE -ne 0) {
        throw "curl failed with exit code $LASTEXITCODE for $Url"
    }
    return ($text -join "`n")
}

function Save-ActiveCurlFile {
    param(
        [string]$Url,
        [string]$Path
    )

    $args = @(
        "--silent", "--show-error", "--fail", "--location", "--http1.1",
        "--connect-timeout", "10",
        "--output", $Path
    )
    if ($script:ActiveInterfaceAddress) {
        $args += @("--interface", $script:ActiveInterfaceAddress)
    }
    $args += $Url

    & curl.exe @args
    if ($LASTEXITCODE -ne 0) {
        throw "curl failed with exit code $LASTEXITCODE for $Url"
    }
}

function Wait-ExportReady {
    param(
        [string]$Base,
        [int]$TimeoutSec = 45
    )

    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSec)
    do {
        try {
            $status = Invoke-ActiveCurlText -Url "$Base/api/status" -TimeoutSec 5 |
                ConvertFrom-Json
            $enrichmentRunning = $false
            if ($status.enrichment) {
                $enrichmentRunning = [bool]$status.enrichment.running
            }
            $storageServiceReady = $status.storage_service -and
                ([string]$status.storage_service.status).StartsWith("export mode active")
            if ($status.app_mode -eq "export" -and
                -not [bool]$status.storage_quiescing -and
                -not $enrichmentRunning -and
                -not [bool]$status.wifi_started -and
                [bool]$status.eth_started -and
                $storageServiceReady) {
                return $status
            }
        } catch {
            # Ethernet remains available during EXPORT, but tolerate the short
            # HTTP restart window before retrying.
        }
        Start-Sleep -Milliseconds 500
    } while ([DateTime]::UtcNow -lt $deadline)

    throw "EXPORT mode did not become ready within $TimeoutSec seconds"
}

function Try-SelectBaseUrl {
    param([string]$Candidate)

    $base = Normalize-BaseUrl $Candidate
    Set-ActiveBaseUrl -Base $base

    if ($script:ActiveInterfaceAddress) {
        Write-Host "Trying $base via local interface $($script:ActiveInterfaceAddress)"
    } else {
        Write-Host "Trying $base"
    }

    if (-not $SkipExportMode) {
        Write-Host "Requesting EXPORT mode..."
        Invoke-ActiveCurlText -Url "$base/api/mode/export?confirm=EXPORT" -Method "POST" -TimeoutSec 20 | Out-Null
        $status = Wait-ExportReady -Base $base
    } elseif ($StopRecordingFirst) {
        Write-Host "Stopping recording before export..."
        Invoke-ActiveCurlText -Url "$base/api/config" -Method "POST" -Body "recording=0" -TimeoutSec 15 | Out-Null
        Start-Sleep -Seconds 2
    }

    if (-not $status) {
        $statusText = Invoke-ActiveCurlText -Url "$base/api/status" -TimeoutSec 15
        $status = $statusText | ConvertFrom-Json
    }
    return [PSCustomObject]@{
        BaseUrl = $base
        InterfaceAddress = $script:ActiveInterfaceAddress
        Status = $status
    }
}

$candidates = @()
$candidates += (Normalize-BaseUrl $BaseUrl)
if ($FallbackBaseUrl) {
    $fallback = Normalize-BaseUrl $FallbackBaseUrl
    if ($fallback -and -not ($candidates -contains $fallback)) {
        $candidates += $fallback
    }
}

$selected = $null
$errors = @()
foreach ($candidate in $candidates) {
    try {
        $selected = Try-SelectBaseUrl -Candidate $candidate
        break
    } catch {
        $errors += "$candidate -> $($_.Exception.Message)"
        Write-Warning "Failed to use ${candidate}: $($_.Exception.Message)"
    }
}

if (-not $selected) {
    throw "No usable board URL found. Tried: $($errors -join '; ')"
}

Set-ActiveBaseUrl -Base $selected.BaseUrl
Write-Host "Using $($selected.BaseUrl)"
if ($selected.InterfaceAddress) {
    Write-Host "Binding curl to local interface address $($selected.InterfaceAddress)"
}

$manifestUrl = "$($selected.BaseUrl)/api/recordings?limit=$Limit"
Write-Host "Fetching $manifestUrl"
$manifest = Invoke-ActiveCurlText -Url $manifestUrl -TimeoutSec 30 | ConvertFrom-Json

$items = @($manifest.recordings) | Where-Object {
    $_.name -and $_.name.EndsWith(".avi") -and -not $_.name.EndsWith(".part")
} | Select-Object -First $Limit

if (-not $items -or $items.Count -eq 0) {
    Write-Host "No finalized .avi recordings found."
    exit 0
}

$results = foreach ($item in $items) {
    $uri = if ($item.uri) { [string]$item.uri } else { "/recording/$($item.name)" }
    if (-not $uri.StartsWith("/")) {
        $uri = "/$uri"
    }
    $url = "$($selected.BaseUrl)$uri"
    $dest = Join-Path $OutDir $item.name

    Write-Host "Downloading $url -> $dest"
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    Save-ActiveCurlFile -Url $url -Path $dest
    $sw.Stop()

    $size = (Get-Item -LiteralPath $dest).Length
    $expected = 0L
    if ($null -ne $item.bytes) {
        $expected = [int64]$item.bytes
    }
    $seconds = [Math]::Max($sw.Elapsed.TotalSeconds, 0.001)
    $mibPerSec = ($size / 1MB) / $seconds

    [PSCustomObject]@{
        Name = $item.name
        Bytes = $size
        ExpectedBytes = $expected
        SizeMatches = ($expected -le 0 -or $size -eq $expected)
        Seconds = [Math]::Round($seconds, 3)
        MiBps = [Math]::Round($mibPerSec, 3)
        Path = (Resolve-Path -LiteralPath $dest).Path
    }
}

$results | Format-Table -AutoSize

if ($results | Where-Object { -not $_.SizeMatches }) {
    Write-Warning "At least one downloaded file size does not match /api/recordings metadata."
    exit 2
}

$avg = ($results | Measure-Object -Property MiBps -Average).Average
Write-Host ("Average throughput: {0:N3} MiB/s" -f $avg)
