# SMC3 R4 USB bridge (smc3_scope 2.8.3): lets the smc3_scope page talk to the board over the USB cable instead
# of WiFi - for calibration (auto-tune, the full check, Seat setup) with SimHub closed. Windows PowerShell 5.1 or
# PowerShell 7, nothing to install. Needs firmware 1.9.5 or newer on the board (1.9.6 recommended).
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

function Smc3-NoQuickEdit {
    # 2.8.2: a click in the old console window starts "Select" (QuickEdit) and freezes the script until Esc - the
    # link would drop. Turns QuickEdit off for this window; returns the mode to put back, or $null.
    try {
        if (-not ('Smc3.Con' -as [type])) {
            Add-Type -Namespace Smc3 -Name Con -MemberDefinition @'
[DllImport("kernel32.dll")] public static extern IntPtr GetStdHandle(int h);
[DllImport("kernel32.dll")] public static extern bool GetConsoleMode(IntPtr h, out uint m);
[DllImport("kernel32.dll")] public static extern bool SetConsoleMode(IntPtr h, uint m);
'@
        }
        $h = [Smc3.Con]::GetStdHandle(-10)
        $m = [uint32]0
        if (-not [Smc3.Con]::GetConsoleMode($h, [ref]$m)) { return $null }
        $nm = [int64]$m -bor 0x80                  # ENABLE_EXTENDED_FLAGS (needed for the change to stick)
        if ($nm -band 0x40) { $nm = $nm - 0x40 }   # ENABLE_QUICK_EDIT_MODE off
        if ([Smc3.Con]::SetConsoleMode($h, [uint32]$nm)) { return $m }
    } catch { }
    return $null
}

function Smc3-PutQuickEdit($mode) {
    if ($null -eq $mode) { return }
    try { [void][Smc3.Con]::SetConsoleMode([Smc3.Con]::GetStdHandle(-10), [uint32]$mode) } catch { }
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

function Smc3-Probe($sp) {
    # 2.8.2: asks the board for its firmware number ([rd!]) right after the port opens - "ok 19", or "none" when
    # nothing at all comes back over USB (then the page can't work over the cable either). 2.8.3: up to 6 times -
    # one answer can get lost while Windows sets the port up (the USB chip clears what it holds each time)
    Start-Sleep -Milliseconds 250
    try { $sp.DiscardInBuffer() } catch { }
    $got = New-Object System.Collections.Generic.List[byte]
    $t0 = Get-Date
    $asked = 0
    $tAsk = [DateTime]::MinValue
    while (((Get-Date) - $t0).TotalMilliseconds -lt 2500) {
        if ($asked -lt 6 -and ((Get-Date) - $tAsk).TotalMilliseconds -ge 350) {
            $sp.Write([byte[]](0x5B, 0x72, 0x64, 0x21, 0x5D), 0, 5)
            $asked++
            $tAsk = Get-Date
        }
        $k = $sp.BytesToRead
        if ($k -gt 0) {
            $b = New-Object byte[] $k
            $n = $sp.Read($b, 0, $k)
            for ($j = 0; $j -lt $n; $j++) { $got.Add($b[$j]) }
            for ($j = 0; $j + 4 -lt $got.Count; $j++) {
                if ($got[$j] -eq 0x5B -and $got[$j + 1] -eq 0x21 -and $got[$j + 4] -eq 0x5D) { return ('ok ' + $got[$j + 3]) }
            }
        }
        Start-Sleep -Milliseconds 30
    }
    if ($got.Count -gt 0) { return ('bytes ' + $got.Count) }
    return 'none'
}

function Smc3-Ask($base, $hdr, [string]$msg) {
    # asks the page whether it wants the cable now (and tells it why the port can't be opened); $null = no answer
    try {
        $body = '{"msg": ' + (ConvertTo-Json $msg) + '}'
        return Invoke-RestMethod -Uri ($base + '/usb/standby') -Method Post -Headers $hdr -Body $body `
                                 -ContentType 'application/json' -TimeoutSec 5
    } catch { return $null }
}

function Smc3-Keep($base, $hdr, [string]$msg, [bool]$viaPage) {
    # 2.8.2: while the bridge holds the port (or keeps trying to) without a link: 'keep' at it, 'free' the port
    # (the switch at the top of the page went back to WiFi), or 'away' (the page doesn't answer)
    $r = Smc3-Ask $base $hdr $msg
    if ($null -eq $r) { return 'away' }
    if ($r.stop) { throw [string]$r.stop }
    if ($viaPage -and -not $r.want) { return 'free' }
    return 'keep'
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
    if ($smc3Port -match '^\d+$') { $smc3Port = 'COM' + $smc3Port }
    if ($smc3Port -notmatch '^COM\d+$') { throw ('"' + $smc3Port + '" is not a COM port (e.g. COM5).') }

    $id = [guid]::NewGuid().ToString('N').Substring(0, 12)
    $hdr = @{ 'X-SMC3' = '1'; 'X-SMC3-Bridge' = $id; 'X-SMC3-Computer' = [string]$env:COMPUTERNAME;
              'X-SMC3-Port' = $smc3Port; 'X-SMC3-Bridge-Version' = 'ps-2.8.3' }
    $kuOff = [byte[]](0x5B, 0x6B, 0x55, 0x00, 0x5D)
    $bufS = New-Object byte[] 4096
    $bufN = New-Object byte[] 4096
    $sp = $null; $tcp = $null
    $want = -not $smc3Standby                  # started by hand: link at once
    $viaPage = $false                          # ... or because the page's switch asked for it (or it has linked)
    $busy = ''; $said = ''; $probe = ''; $away = $null
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
                $want = $true; $viaPage = $true; $said = ''; $away = $null
            }
            if ($null -eq $sp) {
                $p = New-Object System.IO.Ports.SerialPort -ArgumentList $smc3Port, 500000, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
                $p.Handshake = [System.IO.Ports.Handshake]::None
                $p.ReadBufferSize = 65536
                $p.WriteTimeout = 2000                 # 2.8.2: a board that stops taking bytes can't hang the bridge
                $p.DtrEnable = $true
                $p.RtsEnable = $true
                try {
                    $p.Open(); $sp = $p; $busy = ''; $probe = ''
                    # (.NET Framework: pulling the cable while the port is open can crash PowerShell from a finalizer)
                    try { [GC]::SuppressFinalize($sp.BaseStream) } catch { }
                }
                catch {
                    $e = $_.Exception.GetBaseException()
                    if ($e -is [System.UnauthorizedAccessException]) {
                        $busy = $smc3Port + ' is in use - close SimHub (or SMC3Utils, the Arduino IDE''s Serial Monitor).'
                        $tag = 'busy'; $tail = ' It links as soon as the port is free.'
                    } else {                           # 2.8.2: not there (unplugged?) - say what Windows says
                        $busy = 'Can''t open ' + $smc3Port + ': ' + $e.Message.Trim() + ' Is the board plugged in?'
                        $tag = 'gone'; $tail = ' Trying again every 2 s ...'
                    }
                    if ($said -ne $tag) { Smc3-Say ('Waiting: ' + $busy + $tail) Yellow; $said = $tag }
                    if ((Smc3-Keep $base $hdr $busy $viaPage) -eq 'free') {
                        $want = $false; $viaPage = $false; $said = ''               # the switch went back to WiFi
                        continue
                    }
                    Start-Sleep -Seconds 2; continue
                }
            }
            if (-not $probe) {
                try { $probe = Smc3-Probe $sp }
                catch {
                    Smc3-Say ('The USB port stopped working (cable unplugged?): ' + $_.Exception.GetBaseException().Message) Yellow
                    try { $sp.Close() } catch { }
                    $sp = $null; $said = ''
                    if ((Smc3-Keep $base $hdr '' $viaPage) -eq 'free') { $want = $false; $viaPage = $false }
                    Start-Sleep -Seconds 2; continue
                }
                if ($probe -like 'ok *') {
                    $fwn = [int]($probe.Substring(3))
                    Smc3-Say ('The board answers on ' + $smc3Port + ' (firmware ' + [int][math]::Floor($fwn / 10) + '.' + ($fwn % 10) + '.x).') Green
                } elseif ($probe -like 'bytes *') {
                    Smc3-Say ('Something answers on ' + $smc3Port + ', but not like the board - is it the Arduino''s port?') Yellow
                } else {
                    Smc3-Say ('No answer from the board on ' + $smc3Port + ': nothing comes back over USB. Is it the Arduino''s port, and is SimHub (or the Serial Monitor) really closed?') Yellow
                }
            }
            if ($away -and ((Get-Date) - $away).TotalSeconds -gt 60) {
                # 2.8.2: no word from the page for a minute while holding the port - let SimHub have it again
                try { $sp.Close() } catch { }
                $sp = $null; $want = $false; $viaPage = $false; $said = ''; $away = $null
                Smc3-Say ('No word from the page for a minute - letting go of ' + $smc3Port + '.') Yellow
                continue
            }
            $tcp = New-Object System.Net.Sockets.TcpClient
            $tcp.NoDelay = $true
            try { $tcp.Connect($uri.Host, $uri.Port) }
            catch {
                Smc3-Say ('Can''t reach the page at ' + $uri.Authority + ' - trying again in 5 s (is the address right?)') Yellow
                $tcp.Close(); $tcp = $null
                if (-not $away) { $away = Get-Date }
                Start-Sleep -Seconds 5; continue
            }
            $ns = $tcp.GetStream()
            $ns.ReadTimeout = 15000
            $ns.WriteTimeout = 5000                    # 2.8.2
            $req = "POST /usb/link HTTP/1.1`r`nHost: " + $uri.Authority + "`r`nX-SMC3: 1`r`nX-SMC3-Bridge: " + $id +
                   "`r`nX-SMC3-Computer: " + $env:COMPUTERNAME + "`r`nX-SMC3-Port: " + $smc3Port +
                   "`r`nX-SMC3-Bridge-Version: ps-2.8.3`r`nX-SMC3-Probe: " + $probe + "`r`nContent-Length: 0`r`n`r`n"
            $rb = [System.Text.Encoding]::ASCII.GetBytes($req)
            $head = ''
            try {
                $ns.Write($rb, 0, $rb.Length)
                $head = Smc3-Head $ns
            }
            catch {
                Smc3-Say ('No answer from the page (' + $_.Exception.GetBaseException().Message + ') - trying again in 5 s') Yellow
                $tcp.Close(); $tcp = $null
                if (-not $away) { $away = Get-Date }
                Start-Sleep -Seconds 5; continue
            }
            $away = $null
            $code = 0
            if ($head -match '^HTTP/\S+ (\d+)') { $code = [int]$Matches[1] }
            $run = ''
            if ($head -match 'X-SMC3-Run:\s*(\w+)') { $run = $Matches[1] }
            if ($code -ne 200) {
                $msg = ''
                try { $msg = (New-Object System.IO.StreamReader($ns, [System.Text.Encoding]::UTF8)).ReadToEnd().Trim() } catch { }
                $tcp.Close(); $tcp = $null
                if (-not $msg) { $msg = 'The page answered ' + $code + '.' }
                if ($code -eq 409) {
                    Smc3-Say ('Waiting: ' + $msg + ' (trying again in 5 s)') Yellow
                    $keep = Smc3-Keep $base $hdr '' $viaPage    # 2.8.2: let go if the switch went back to WiFi meanwhile
                    if ($keep -eq 'free') {
                        try { $sp.Close() } catch { }
                        $sp = $null; $want = $false; $viaPage = $false; $said = ''
                        continue
                    }
                    if ($keep -eq 'away' -and -not $away) { $away = Get-Date }
                    Start-Sleep -Seconds 5; continue
                }
                if ($code -eq 403) {                   # the page is set to WiFi only: let go of the port and wait
                    Smc3-Say $msg Yellow
                    try { $sp.Close() } catch { }
                    $sp = $null; $want = $false; $viaPage = $false; $said = ''
                    continue
                }
                throw $msg
            }
            $ns.ReadTimeout = [System.Threading.Timeout]::Infinite
            Smc3-Say ''
            Smc3-Say 'LINKED: the page now talks to the board over this USB cable (no WiFi).' Green
            Smc3-Say 'Calibrate in the page. When you are done, flip the switch at the top of the page to WiFi (or close this window), then start SimHub.' Green
            # 2.8.2: the page's side waits on the network (wakes at once when the page sends); the board's side is read
            # whenever bytes are waiting (checked about every 10 ms) - an asynchronous read of the COM port never
            # completed on some PCs, so nothing came back from the board
            $viaPage = $true                           # (2.8.2: from now on the page's switch decides)
            $tN = $ns.ReadAsync($bufN, 0, $bufN.Length)
            $up = 0; $down = 0; $since = Get-Date; $shown = Get-Date; $heard = Get-Date; $why = ''; $usbLost = $false
            while ($true) {
                $done = $false
                try { $done = $tN.Wait(10) } catch { $done = $true }
                if ($done) {
                    $n = 0
                    try { $n = $tN.Result } catch { $n = -1 }
                    if ($n -le 0) { $why = 'The page ended the link.'; break }
                    try { $sp.Write($bufN, 0, $n) }
                    catch { $why = 'The USB port stopped working (cable unplugged?): ' + $_.Exception.GetBaseException().Message; $usbLost = $true; break }
                    $down += $n
                    $heard = Get-Date
                    $tN = $ns.ReadAsync($bufN, 0, $bufN.Length)
                }
                $n = 0
                try {
                    $k = $sp.BytesToRead
                    if ($k -gt 0) { $n = $sp.Read($bufS, 0, [Math]::Min($k, $bufS.Length)) }
                }
                catch { $why = 'The USB port stopped working (cable unplugged?): ' + $_.Exception.GetBaseException().Message; $usbLost = $true; break }
                if ($n -gt 0) {
                    try { $ns.Write($bufS, 0, $n) } catch { $why = 'The connection to the page dropped.'; break }
                    $up += $n
                }
                $now = Get-Date
                # (the page sends [kU1] every second; 10 s leaves room for a page busy with a WiFi reconnect)
                if (($now - $heard).TotalSeconds -gt 10) { $why = 'The page went quiet for 10 s (network down, or the NAS off?).'; break }
                if (($now - $shown).TotalSeconds -ge 2) {
                    $shown = $now
                    $t = [int]($now - $since).TotalSeconds
                    Write-Host -NoNewline ("`r  linked {0}:{1:00}   board -> page {2} kB   page -> board {3} kB   " -f [int][math]::Floor($t / 60), ($t % 60), [int]($up / 1024), [int]($down / 1024))
                }
            }
            Write-Host ''
            if (-not $usbLost) { try { $sp.Write($kuOff, 0, $kuOff.Length) } catch { } }
            try { $tcp.Close() } catch { }
            $tcp = $null
            Smc3-Say $why Yellow
            if ($usbLost) {
                # 2.8.2: let go of the dead port and open it again when the board is back (plug the cable in again)
                try { $sp.Close() } catch { }
                $sp = $null; $said = ''
                Start-Sleep -Seconds 2
                continue
            }
            Start-Sleep -Milliseconds 500
            $r = Smc3-Ask $base $hdr ''
            if ($null -eq $r) {                        # the page went away (restarting?): keep the port a minute
                Smc3-Say 'The page doesn''t answer - linking again as soon as it does ...' Yellow
                $until = (Get-Date).AddSeconds(60)
                while ($null -eq $r -and (Get-Date) -lt $until) { Start-Sleep -Seconds 2; $r = Smc3-Ask $base $hdr '' }
            }
            if ($r -and $r.stop) { throw [string]$r.stop }
            if ($r -and ($r.want -or ($run -and $r.run -and $r.run -ne $run))) {
                # dropped, but the page still wants the cable - or it has restarted meanwhile: link again
                # (2.8.2: back after a network blip with the switch on WiFi meanwhile = let go, as below)
                Smc3-Say 'Linking again ...' Yellow
                continue
            }
            try { $sp.Close() } catch { }
            $sp = $null; $want = $false; $viaPage = $false; $said = ''
            if ($r) { Smc3-Say 'The page is back on WiFi.' Yellow } else { Smc3-Say 'The page is still away - letting go of the port.' Yellow }
        }
    }
    finally {
        if ($sp) {
            try { $sp.Write($kuOff, 0, $kuOff.Length) } catch { }
            try { $sp.Close() } catch { }
        }
        if ($tcp) { try { $tcp.Close() } catch { } }   # 2.8.2: the page sees "bridge closed" at once
    }
}

$smc3Mode = Smc3-NoQuickEdit
try { Smc3-Bridge }
catch { Write-Host ''; Smc3-Say ('Stopped: ' + $_.Exception.Message) Red }
finally { Smc3-PutQuickEdit $smc3Mode }       # (also after Ctrl+C, for a console that stays open)
Read-Host 'Press Enter to close this window' | Out-Null
