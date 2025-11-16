param(
  [string]$ClientName = "win-client-01",
  [string]$ServerPublicKey = "__SERVER_PUBLIC_KEY__",
  [string]$PresharedKey = "",   # deja vacío si no usas PSK
  [string]$Address = "10.6.0.10/32",
  [string]$DNS = "10.6.0.1",
  [string]$AllowedIPs = "10.6.0.0/24, 192.168.50.0/24",
  [string]$Endpoint = "vpn.tu-dominio.com:51820"
)

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$keysDir = Join-Path $here "keys\$ClientName"
$template = Get-Content (Join-Path $here "client.template.conf") -Raw

# Asegura herramientas de WireGuard en PATH (wg.exe viene con WireGuard para Windows)
New-Item -ItemType Directory -Force -Path $keysDir | Out-Null

# Genera llaves
$priv = & wg.exe genkey
$pub  = $priv | & wg.exe pubkey

# Rellena template
$conf = $template `
  -replace "__CLIENT_PRIVATE_KEY__", $priv `
  -replace "__SERVER_PUBLIC_KEY__", $ServerPublicKey `
  -replace "10.6.0.10/32", $Address `
  -replace "10.6.0.1", $DNS `
  -replace "10.6.0.0/24, 192.168.50.0/24", $AllowedIPs `
  -replace "vpn.tu-dominio.com:51820", $Endpoint

if ($PresharedKey -ne "") {
  $conf = $conf -replace "# PresharedKey = __PSK__", "PresharedKey = $PresharedKey"
} else {
  $conf = $conf -replace "# PresharedKey = __PSK__\r?\n", ""
}

# Guarda artefactos locales (no versionar)
$confPath = Join-Path $keysDir "$ClientName.conf"
$privPath = Join-Path $keysDir "$ClientName.priv"
$pubPath  = Join-Path $keysDir "$ClientName.pub"

$priv | Out-File -NoNewline -Encoding ASCII $privPath
$pub  | Out-File -NoNewline -Encoding ASCII $pubPath
$conf | Out-File -Encoding ASCII $confPath

Write-Host "Listo:"
Write-Host "  Config: $confPath"
Write-Host "  PublicKey (dásela al servidor): $(Get-Content $pubPath)"
