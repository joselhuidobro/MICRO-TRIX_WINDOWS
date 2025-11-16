<#
.SYNOPSIS
    Configura Windows Defender Firewall para el stack Trix
.DESCRIPTION
    Bloquea TODO excepto WireGuard. Lee puertos desde .ports.env
.NOTES
    Ejecutar como Administrador. Es IDEMPOTENTE (puedes ejecutar cuantas veces quieras)
#>

# --- 1. CARGAR VARIABLES ---
$ErrorActionPreference = "Stop"
$PortsFile = Join-Path $PSScriptRoot ".ports.env"

if (-not (Test-Path $PortsFile)) {
    Write-Error "❌ No existe .ports.env. Crea el archivo primero."
    exit 1
}

Get-Content $PortsFile | ForEach-Object {
    if ($_ -match '^\s*([^#=]+)=(.+)$') {
        $name = $matches[1].Trim()
        $value = $matches[2].Trim()
        Set-Variable -Name $name -Value $value -Scope Script
    }
}

# --- 2. LIMPIAR REGLAS ANTIGUAS ---
Write-Host "🧹 Limpiando reglas antiguas..."
Get-NetFirewallRule -DisplayName "trix-*" -ErrorAction SilentlyContinue | Remove-NetFirewallRule

# --- 3. CONFIGURAR PERFILES ---
Write-Host "🔒 Configurando perfiles de firewall..."
Set-NetFirewallProfile -Profile Public -DefaultInboundAction Block -DefaultOutboundAction Allow -Enabled True
Set-NetFirewallProfile -Profile Private -DefaultInboundAction Allow -DefaultOutboundAction Allow -Enabled True

# --- 4. REGLAS ESPECÍFICAS ---

# WireGuard (único puerto abierto al mundo)
New-NetFirewallRule `
    -DisplayName "trix-WireGuard-IN" `
    -Description "Permite conexiones WireGuard VPN" `
    -Direction Inbound `
    -Protocol UDP `
    -LocalPort $WIREGUARD_PORT `
    -Action Allow `
    -Profile Public `
    -Enabled True

New-NetFirewallRule `
    -DisplayName "trix-Docker-BYPASS-FIX" `
    -Description "BLOQUEA puertos Docker que se filtran (Kong, etc.)" `
    -Direction Inbound `
    -Protocol TCP `
    -LocalPort @($KONG_PORT, $KONG_STATUS_PORT, "80", "443") `
    -Action Block `
    -Profile Public `
    -Enabled True

# --- 6. VALIDACIÓN ---
Write-Host ""
Write-Host "✅ REGLAS APLICADAS:"
Get-NetFirewallRule -DisplayName "trix-*" | Format-Table DisplayName, Direction, Action, Enabled -AutoSize

Write-Host ""
Write-Host "🔍 TEST: Intenta acceder a http://tu-ip-publica:$KONG_PORT desde fuera de tu red"
Write-Host "   Debe FALLAR (timeout). Si funciona, el firewall no está bloqueando."

# --- 7. PERSISTENCIA ---
# Exporta configuración por si acaso
$backupFile = "firewall-backup-$(Get-Date -Format 'yyyyMMdd-HHmmss').wfw"
netsh advfirewall export $backupFile
Write-Host "💾 Backup guardado: $backupFile"