# Builds the RobotsCRX Rhino package (.yak) into artifacts/yak.
# Usage: pwsh build/pack-crx.ps1 [-SkipTests]
# Requires the .NET SDK pinned in global.json and Rhino 8 (for its Yak tool).

param([switch]$SkipTests)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$artifacts = Join-Path $root 'artifacts'
# A separate output folder, so a running Rhino holding the development build does not block packaging.
$build = Join-Path $artifacts 'pack'
$yakFolder = Join-Path $artifacts 'yak'
$stage = Join-Path $yakFolder 'stage'

function Invoke-Checked([string]$file, [string[]]$arguments) {
    & $file @arguments

    if ($LASTEXITCODE -ne 0) {
        throw "'$file $arguments' failed with exit code $LASTEXITCODE."
    }
}

$props = [xml](Get-Content (Join-Path $root 'src/Package.props') -Raw)
$version = $props.Project.PropertyGroup.Version
$url = $props.Project.PropertyGroup.PackageProjectUrl

if (-not $version -or -not $url) {
    throw 'Version and PackageProjectUrl must be set in src/Package.props.'
}

$yak = @(
    'C:\Program Files\Rhino 8\System\Yak.exe',
    '/Applications/Rhino 8.app/Contents/Resources/bin/yak'
) | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $yak) {
    throw 'Rhino 8 Yak tool not found.'
}

if (-not $SkipTests) {
    Invoke-Checked 'dotnet' @('test', (Join-Path $root 'tests/Robots.Tests'), '-f', 'net10.0', '-c', 'Release', "-p:ArtifactsPath=$build")
}

Invoke-Checked 'dotnet' @('build', (Join-Path $root 'src/Robots.Grasshopper'), '-c', 'Release', "-p:ArtifactsPath=$build")

Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
$output = Join-Path $build 'bin/Robots.Grasshopper/release'
Copy-Item $output $stage -Recurse
Get-ChildItem $stage -Recurse -Include '*.pdb', '*.deps.json' | Remove-Item

# Robots is MIT licensed; the license and third-party notices travel with redistributed copies.
Copy-Item (Join-Path $root 'LICENSE'), (Join-Path $root 'NOTICE') $stage

$manifest = @"
---
name: RobotsCRX
version: $version
authors:
- Robots Authors (Vicente Soler, visose)
- Sam Wilcock
description: Fork of Robots (https://github.com/visose/Robots) with FANUC CRX robots, Stream Motion streaming and a Read Joints component. Create and simulate ABB, KUKA, UR, Staubli, Franka Emika, Doosan, Fanuc, Igus, and Jaka robot programs in Rhino 8 and Grasshopper. Replaces the Robots package; uninstall Robots first.
url: $url
icon: robots-icon.png
"@

# Without a byte order mark, which Windows PowerShell adds for -Encoding utf8.
[IO.File]::WriteAllText((Join-Path $stage 'manifest.yml'), $manifest, [Text.UTF8Encoding]::new($false))

# Rhino's Yak targets .NET 8; allow a newer runtime when only that is installed.
$env:DOTNET_ROLL_FORWARD = 'Major'
Push-Location $stage

try {
    Invoke-Checked $yak @('build', '--platform', 'any')
}
finally {
    Pop-Location
    Remove-Item Env:DOTNET_ROLL_FORWARD
}

Get-ChildItem $yakFolder -Filter '*.yak' | Remove-Item
$package = Get-ChildItem $stage -Filter '*.yak' | Move-Item -Destination $yakFolder -PassThru
Remove-Item $stage -Recurse -Force
Write-Host "Package: $($package.FullName)"
