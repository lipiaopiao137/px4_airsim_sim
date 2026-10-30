#!/usr/bin/env bash
# Start AirSim Blocks on Windows and wait until its two TCP listeners are up.
#
# AirSim is the TCP *server* on 4560 for simulator_mavlink.  Never connect to
# 4560 to "test" it: AirSim accepts the first peer as its SITL client, so the
# probe steals the connection and PX4 can never attach.  Only inspect the
# Listen state (Get-NetTCPConnection -State Listen).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

case "$(blocks_state)" in
    ready)
        log "Blocks already ready (41451 and 4560 listening; waiting for PX4)"
        exit 0
        ;;
    connected)
        log "Blocks already running and connected to PX4 on 4560"
        log "note: to restart PX4, close Blocks first and run this script again"
        exit 0
        ;;
    rpc)
        die "Blocks RPC is up but 4560 is not listening (stale SITL link); close Blocks on Windows and re-run"
        ;;
esac

log "starting AirSim Blocks (windowed 1280x720, log: $BLOCKS_LOG)"
if ! powershell.exe -NoProfile -NonInteractive -Command - <<'PS'
$exe = 'D:\AirSim\Blocks\Blocks\WindowsNoEditor\Blocks.exe'
$wd  = 'D:\AirSim\Blocks\Blocks\WindowsNoEditor'
$log = 'D:\AirSim\blocks_abs.log'

$p = Get-Process -Name Blocks -ErrorAction SilentlyContinue
if (-not $p) {
    Start-Process -FilePath $exe -WorkingDirectory $wd `
        -ArgumentList @('-windowed','-ResX=1280','-ResY=720','-log',("-abslog=" + $log))
    Write-Output 'launched Blocks'
} else {
    Write-Output ("Blocks already running (pid {0})" -f $p[0].Id)
}

$deadline = (Get-Date).AddSeconds(180)
while ((Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 2
    $rpc  = Get-NetTCPConnection -LocalPort 41451 -State Listen -ErrorAction SilentlyContinue
    $sitl = Get-NetTCPConnection -LocalPort 4560 -State Listen -ErrorAction SilentlyContinue
    if ($rpc -and $sitl) {
        $proc = Get-Process -Name Blocks -ErrorAction SilentlyContinue
        Write-Output ("Blocks READY (pid {0}); 41451 and 4560 listening" -f $proc[0].Id)
        exit 0
    }
}
Write-Error 'Blocks did not open 41451/4560 within 180 s'
exit 1
PS
then
    log "Blocks did not become ready; last lines of $BLOCKS_LOG:"
    tail -n 20 /mnt/d/AirSim/blocks_abs.log 2>/dev/null || true
    die "if Blocks is already running but broken, close it on Windows and re-run"
fi

log "Blocks ready; 4560 is listening (not probed)"
