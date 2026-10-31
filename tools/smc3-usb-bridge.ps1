# SMC3 R4 USB bridge (smc3_scope 2.8.1): lets the smc3_scope page talk to the board over the USB cable instead
# of WiFi - for calibration (auto-tune, the full check, Seat setup) with SimHub closed. Windows PowerShell 5.1 or
# PowerShell 7, nothing to install. Needs firmware 1.9.5 or newer on the board.
#   From the page: Settings, USB cable - press Win+R and paste the line shown there.
#   As a file:     powershell -NoProfile -ExecutionPolicy Bypass -File smc3-usb-bridge.ps1 [http://<page>:<port>] [COMx] [standby]
# It links at once. When the page's switch goes back to WiFi it lets go of the port (SimHub can have it) and waits:
# flip the switch at the top of the page to USB and it links again. "standby" (or SMC3_STANDBY=1) starts it waiting.
# Close this window (or press Ctrl+C) to stop it.

$ErrorActionPreference = 'Stop'
[System.Net.ServicePointManager]::Expect100Continue = $false     # (the page answers at once; don't wait for a 100)
$ProgressPreference = 'SilentlyContinue'                            # (no progress bar for every question to the page)
$smc3Page = ''
$smc3Port = ''
$smc3Standby = ($env:SMC3_STANDBY -eq '1')
foreach ($smc3Arg in $args) {
    if ($smc3Arg -eq 'standby') { $smc3Standby = $true }
    elseif ([string]$smc3Arg -match '^(?i)COM\d+$') { $smc3Port = ([string]$smc3Arg).ToUpper() }
    elseif ($smc3Arg) { $smc3Page = [string]$smc3Arg }
}
if ($smc3Page -eq '' -or $smc3Page -like '*PAGE__*') { $smc3Page = '' }

function Smc3-Say([string]$text, [string]$colour) {
    if (-not $colour) { $colour = 'Gray' }
    Write-Host $text -ForegroundColor $colour
}

function Smc3-Ports {
    $list = @()
    try {
        $list = @(Get-CimInstance -ClassName Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction Stop | ForEach-Object {
            if ([string]$_.Name -match '\((COM\d+)\)') {
                New-Object PSObject -Property @{ Port = $Matches[1]; Name = [string]$_.Name;
                                                 Arduino = ([string]$_.PNPDeviceID -match 'VID_2341') }
            }
        })
    } catch { $list = @() }
    if ($list.Count -eq 0) {
        $list = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique | ForEach-Object {
            New-Object PSObject -Property @{ Port = $_; Name = $_; Arduino = $false }
        })
    }
    return ,$list
}

function Smc3-Head($ns) {
    # the page's answer: status line and headers (byte by byte, so nothing after them is read)
    $head = New-Object System.Text.StringBuilder
    while (-not $head.ToString().EndsWith("`r`n`r`n")) {
        $c = $ns.ReadByte()
        if ($c -lt 0) { throw 'The page closed the connection.' }
        [void]$head.Append([char]$c)
        if ($head.Length -gt 16384) { throw 'That is not the smc3_scope page.' }
    }
    return $head.ToString()
}

function Smc3-Ask($base, $hdr, [string]$msg) {
    # asks the page whether it wants the cable now (and tells it why the port can't be opened); $null = no answer
    try {
        $body = '{"msg": ' + (ConvertTo-Json $msg) + '}'
        return Invoke-RestMethod -Uri ($base + '/usb/standby') -Method Post -Headers $hdr -Body $body `
                                 -ContentType 'application/json' -TimeoutSec 5
    } catch { return $null }
}

function Smc3-Bridge {
    Smc3-Say 'SMC3 R4 USB bridge - the page talks to the board over this USB cable instead of WiFi.' Cyan
    if (-not $smc3Page) {
        $smc3Page = (Read-Host 'Address of the smc3_scope page (as in your browser, e.g. http://192.168.1.20:8088)').Trim()
    }
    if ($smc3Page -notmatch '^https?://') { $smc3Page = 'http://' + $smc3Page }
    $uri = [Uri]$smc3Page
    if ($uri.Scheme -ne 'http') { throw 'The bridge needs the page''s plain http:// address (not https or a reverse proxy).' }
    $base = $uri.GetLeftPart([System.UriPartial]::Authority)

    if (-not $smc3Port) {
        $ports = Smc3-Ports
        $ard = @($ports | Where-Object { $_.Arduino })
        if ($ard.Count -eq 1) { $smc3Port = $ard[0].Port }
        elseif ($ports.Count -eq 1) { $smc3Port = $ports[0].Port }
        elseif ($ports.Count -eq 0) { throw 'No COM port on this PC. Is the board plugged in with its USB cable?' }
        else {
            Smc3-Say 'COM ports on this PC:'
            foreach ($p in $ports) {
                $tag = ''
                if ($p.Arduino) { $tag = '   <- Arduino' }
                Smc3-Say ('  ' + $p.Name + $tag)
            }
            $smc3Port = (Read-Host 'Which one is the board (e.g. COM5)').Trim().ToUpper()
        }
    }

    $id = [guid]::NewGuid().ToString('N').Substring(0, 12)
    $hdr = @{ 'X-SMC3' = '1'; 'X-SMC3-Bridge' = $id; 'X-SMC3-Computer' = [string]$env:COMPUTERNAME;
              'X-SMC3-Port' = $smc3Port; 'X-SMC3-Bridge-Version' = 'ps-2.8.1' }
    $kuOff = [byte[]](0x5B, 0x6B, 0x55, 0x00, 0x5D)
    $bufS = New-Object byte[] 4096
    $bufN = New-Object byte[] 4096
    $sp = $null; $ss = $null; $tS = $null
    $want = -not $smc3Standby                  # started by hand: link at once
    $viaPage = $false                          # ... or because the page's switch asked for it
    $busy = ''; $said = ''
    Smc3-Say ('Board: ' + $smc3Port + ' at 500000 baud.   Page: ' + $uri.Authority) Gray
    try {
        while ($true) {
            if (-not $want) {
                # waiting: the port stays closed (SimHub can use it) until the page's switch asks for the cable
                if ($said -ne 'standby') {
                    Smc3-Say ('Waiting with ' + $smc3Port + ' free for SimHub. Flip the switch at the top of the page to USB ' +
                              'to link again - or close this window.') Cyan
                    $said = 'standby'
                }
                $r = Smc3-Ask $base $hdr ''
                if ($null -eq $r) {
                    if ($said -ne 'nopage') { Smc3-Say ('Can''t reach the page at ' + $uri.Authority + ' - trying again ...') Yellow; $said = 'nopage' }
                    Start-Sleep -Seconds 2; continue
                }
                if ($r.stop) { throw [string]$r.stop }
                if (-not $r.want) { if ($said -eq 'nopage') { $said = '' }; Start-Sleep -Seconds 1; continue }
                $want = $true; $viaPage = $true; $said = ''
            }
            if ($null -eq $sp) {
                $p = New-Object System.IO.Ports.SerialPort -ArgumentList $smc3Port, 500000, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
                $p.Handshake = [System.IO.Ports.Handshake]::None
                $p.ReadBufferSize = 65536
                $p.DtrEnable = $true
                $p.RtsEnable = $true
                try { $p.Open(); $sp = $p; $ss = $sp.BaseStream; $tS = $null; $busy = '' }
                catch {
                    $busy = $smc3Port + ' is in use - close SimHub (or SMC3Utils, the Arduino IDE''s Serial Monitor).'
                    if ($said -ne 'busy') { Smc3-Say ('Waiting: ' + $busy + ' It links as soon as the port is free.') Yellow; $said = 'busy' }
                    $r = Smc3-Ask $base $hdr $busy
                    if ($r -and $r.stop) { throw [string]$r.stop }
                    if ($viaPage -and $r -and -not $r.want) { $want = $false }    # the switch went back to WiFi
                    Start-Sleep -Seconds 2; continue
                }
            }
            $tcp = New-Object System.Net.Sockets.TcpClient
            $tcp.NoDelay = $true
            try { $tcp.Connect($uri.Host, $uri.Port) }
            catch {
                Smc3-Say ('Can''t reach the page at ' + $uri.Authority + ' - trying again in 5 s (is the address right?)') Yellow
                $tcp.Close(); Start-Sleep -Seconds 5; continue
            }
            $ns = $tcp.GetStream()
            $ns.ReadTimeout = 15000
            $req = "POST /usb/link HTTP/1.1`r`nHost: " + $uri.Authority + "`r`nX-SMC3: 1`r`nX-SMC3-Bridge: " + $id +
                   "`r`nX-SMC3-Computer: " + $env:COMPUTERNAME + "`r`nX-SMC3-Port: " + $smc3Port +
                   "`r`nX-SMC3-Bridge-Version: ps-2.8.1`r`nContent-Length: 0`r`n`r`n"
            $rb = [System.Text.Encoding]::ASCII.GetBytes($req)
            $head = ''
            try {
                $ns.Write($rb, 0, $rb.Length)
                $head = Smc3-Head $ns
            }
            catch {
                Smc3-Say ('No answer from the page (' + $_.Exception.GetBaseException().Message + ') - trying again in 5 s') Yellow
                $tcp.Close(); Start-Sleep -Seconds 5; continue
            }
            $code = 0
            if ($head -match '^HTTP/\S+ (\d+)') { $code = [int]$Matches[1] }
            $run = ''
            if ($head -match 'X-SMC3-Run:\s*(\w+)') { $run = $Matches[1] }
            if ($code -ne 200) {
                $msg = ''
                try { $msg = (New-Object System.IO.StreamReader($ns, [System.Text.Encoding]::UTF8)).ReadToEnd().Trim() } catch { }
                $tcp.Close()
                if (-not $msg) { $msg = 'The page answered ' + $code + '.' }
                if ($code -eq 409) { Smc3-Say ('Waiting: ' + $msg + ' (trying again in 5 s)') Yellow; Start-Sleep -Seconds 5; continue }
                if ($code -eq 403) {                   # the page is set to WiFi only: let go of the port and wait
                    Smc3-Say $msg Yellow
                    try { $sp.Close() } catch { }
                    $sp = $null; $ss = $null; $tS = $null; $want = $false; $said = ''
                    continue
                }
                throw $msg
            }
            $ns.ReadTimeout = [System.Threading.Timeout]::Infinite
            Smc3-Say ''
            Smc3-Say 'LINKED: the page now talks to the board over this USB cable (no WiFi).' Green
            Smc3-Say 'Calibrate in the page. When you are done, flip the switch at the top of the page to WiFi (or close this window), then start SimHub.' Green
            if ($null -eq $tS) { $tS = $ss.ReadAsync($bufS, 0, $bufS.Length) }
            $tN = $ns.ReadAsync($bufN, 0, $bufN.Length)
            $up = 0; $down = 0; $since = Get-Date; $shown = Get-Date; $heard = Get-Date; $why = ''
            while ($true) {
                $i = [System.Threading.Tasks.Task]::WaitAny([System.Threading.Tasks.Task[]]@($tS, $tN), 500)
                if ($i -eq 0) {
                    $n = 0
                    try { $n = $tS.Result } catch { throw ('The USB port stopped working (cable unplugged?): ' + $_.Exception.GetBaseException().Message) }
                    if ($n -le 0) { throw 'The USB port closed.' }
                    try { $ns.Write($bufS, 0, $n) } catch { $why = 'The connection to the page dropped.'; $tS = $ss.ReadAsync($bufS, 0, $bufS.Length); break }
                    $up += $n
                    $tS = $ss.ReadAsync($bufS, 0, $bufS.Length)
                } elseif ($i -eq 1) {
                    $n = 0
                    try { $n = $tN.Result } catch { $n = -1 }
                    if ($n -le 0) { $why = 'The page ended the link.'; break }
                    $ss.Write($bufN, 0, $n)
                    $down += $n
                    $heard = Get-Date
                }
                $now = Get-Date
                if (($now - $heard).TotalSeconds -gt 6) { $why = 'The page went quiet for 6 s (network down, or the NAS off?).'; break }
                if (($now - $shown).TotalSeconds -ge 2) {
                    $shown = $now
                    $t = [int]($now - $since).TotalSeconds
                    Write-Host -NoNewline ("`r  linked {0}:{1:00}   board -> page {2} kB   page -> board {3} kB   " -f [int][math]::Floor($t / 60), ($t % 60), [int]($up / 1024), [int]($down / 1024))
                }
            }
            Write-Host ''
            try { $ss.Write($kuOff, 0, $kuOff.Length) } catch { }
            $tcp.Close()
            Smc3-Say $why Yellow
            Start-Sleep -Milliseconds 500
            $r = Smc3-Ask $base $hdr ''
            if ($null -eq $r) {                        # the page went away (restarting?): keep the port a minute
                Smc3-Say 'The page doesn''t answer - linking again as soon as it does ...' Yellow
                $until = (Get-Date).AddSeconds(60)
                while ($null -eq $r -and (Get-Date) -lt $until) { Start-Sleep -Seconds 2; $r = Smc3-Ask $base $hdr '' }
                if ($r -and -not $r.stop) { Smc3-Say 'Linking again ...' Yellow; continue }
            }
            if ($r -and $r.stop) { throw [string]$r.stop }
            if ($r -and ($r.want -or ($run -and $r.run -and $r.run -ne $run))) {
                # dropped, but the page still wants the cable - or it has restarted meanwhile: link again
                Smc3-Say 'Linking again ...' Yellow
                continue
            }
            try { $sp.Close() } catch { }
            $sp = $null; $ss = $null; $tS = $null; $want = $false; $viaPage = $false; $said = ''
            if ($r) { Smc3-Say 'The page is back on WiFi.' Yellow } else { Smc3-Say 'The page is still away - letting go of the port.' Yellow }
        }
    }
    finally {
        if ($sp) {
            try { $ss.Write($kuOff, 0, $kuOff.Length) } catch { }
            try { $sp.Close() } catch { }
        }
    }
}

try { Smc3-Bridge }
catch { Write-Host ''; Smc3-Say ('Stopped: ' + $_.Exception.Message) Red }
Read-Host 'Press Enter to close this window' | Out-Null
