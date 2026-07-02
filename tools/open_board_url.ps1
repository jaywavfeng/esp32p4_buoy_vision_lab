param(
    [string]$HostName = "p4-buoy.local",
    [string]$FallbackIp = "169.254.100.2",
    [string]$InterfaceAddress = "",
    [int]$TimeoutSec = 5,
    [switch]$Open
)

$ErrorActionPreference = "Stop"
if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
    $PSNativeCommandUseErrorActionPreference = $false
}

function Get-ApipaAddresses {
    $items = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -like "169.254.*" -and $_.InterfaceAlias -notlike "*Bluetooth*" })
    $preferred = @($items | Where-Object {
        $_.InterfaceAlias -match "Ethernet|Local Area" -or $_.InterfaceIndex -eq 10
    } | Sort-Object InterfaceIndex)
    $others = @($items | Where-Object {
        $_.InterfaceAlias -notmatch "Ethernet|Local Area" -and $_.InterfaceIndex -ne 10
    } | Sort-Object InterfaceIndex)
    return @(($preferred + $others) | Select-Object -ExpandProperty IPAddress)
}

function Test-BoardUrl {
    param(
        [string]$Ip,
        [string]$Interface
    )

    $url = "http://$Ip/api/status"
    $args = @("--max-time", "$TimeoutSec", "--fail", "-sS")
    if ($Interface) {
        $args += @("--interface", $Interface)
    }
    $args += $url

    $oldPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    $out = & curl.exe @args 2>$null
    $exitCode = $LASTEXITCODE
    $ErrorActionPreference = $oldPreference
    if ($exitCode -eq 0 -and $out -match '"target":"esp32p4"') {
        return $true
    }
    return $false
}

$resolvedIp = ""
try {
    $resolvedIp = (Resolve-DnsName $HostName -Type A -ErrorAction Stop |
        Select-Object -First 1 -ExpandProperty IPAddress)
} catch {
    $resolvedIp = ""
}

$candidates = @()
if ($resolvedIp) {
    $candidates += $resolvedIp
}
if ($FallbackIp -and $FallbackIp -notin $candidates) {
    $candidates += $FallbackIp
}

foreach ($ip in $candidates) {
    $interfaces = @("")
    if ($InterfaceAddress) {
        $interfaces = @($InterfaceAddress)
    } elseif ($ip -like "169.254.*") {
        $interfaces = (Get-ApipaAddresses) + @("")
    }
    foreach ($interface in $interfaces) {
        if (Test-BoardUrl -Ip $ip -Interface $interface) {
            $url = "http://$ip/"
            if ($interface) {
                Write-Output "$url  # interface $interface"
            } else {
                Write-Output $url
            }
            if ($Open) {
                Start-Process $url
            }
            exit 0
        }
    }
}

Write-Error "Board not reachable. Try: .\tools\open_board_url.ps1 -InterfaceAddress <your 169.254.x.x>"
