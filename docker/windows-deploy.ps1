param(
  [switch]$Preview,
  [switch]$SkipImageBuild,
  [string]$ImageTar = "",
  [double]$RuntimeSec = 10,
  [string]$RunRoot = ".docker-runs"
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

if ($PSVersionTable.PSVersion.Major -ge 7) {
  $PSNativeCommandUseErrorActionPreference = $false
}

$dockerCli = "C:\Program Files\Docker\Docker\resources\bin\docker.exe"
$dockerUserDir = Join-Path $env:USERPROFILE ".docker"
$dockerContextsDir = Join-Path $dockerUserDir "contexts"
$dockerSessionDir = Join-Path $env:TEMP ("bsp_webots_autoaim_docker_" + [guid]::NewGuid().ToString("N"))
$dockerSafePath = "C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0"

$repoRoot = Split-Path -Parent $PSScriptRoot
$controllerPath = Join-Path $repoRoot "build\rm_auto_aim"
Set-Location $repoRoot

if (!(Test-Path $dockerCli)) {
  throw "docker.exe not found at $dockerCli"
}

if (!(Test-Path $dockerContextsDir)) {
  throw "Docker contexts directory not found at $dockerContextsDir"
}

New-Item -ItemType Directory -Force -Path $dockerSessionDir | Out-Null
Copy-Item -Recurse -Force $dockerContextsDir $dockerSessionDir
Set-Content -Path (Join-Path $dockerSessionDir "config.json") -Encoding Ascii -Value '{"auths":{},"currentContext":"desktop-linux"}'

$env:DOCKER_CONFIG = $dockerSessionDir
$env:HOME = $dockerSessionDir
$env:USERPROFILE = $dockerSessionDir
$env:PATH = $dockerSafePath

function Invoke-DockerCli {
  param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Args
  )

  & $dockerCli @Args
  if ($LASTEXITCODE -ne 0) {
    throw "docker command failed: $dockerCli $($Args -join ' ')"
  }
}

try {
  if ($ImageTar -ne "") {
    Invoke-DockerCli load -i $ImageTar
  }

  if (-not $SkipImageBuild) {
    Invoke-DockerCli compose build
  }

  if ((-not $Preview) -or (-not (Test-Path $controllerPath))) {
    Invoke-DockerCli compose run --rm autoaim-build
  }

  if ($Preview) {
    $env:XR_RUNTIME_SEC = [string]$RuntimeSec
    $containerRunRoot = ($RunRoot -replace "\\", "/")
    if ($containerRunRoot.StartsWith("./")) {
      $containerRunRoot = $containerRunRoot.Substring(2)
    }
    $env:XR_RUN_ROOT = "/workspace/" + $containerRunRoot.TrimStart('/')
    Invoke-DockerCli compose run --rm autoaim-preview
  }
} finally {
  Remove-Item -Recurse -Force $dockerSessionDir -ErrorAction SilentlyContinue
}
