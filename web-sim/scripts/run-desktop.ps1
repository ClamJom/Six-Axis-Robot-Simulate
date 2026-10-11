# 六轴机械臂仿真 —— 桌面端启动脚本（Electron）
# 用法： pwsh -File scripts\run-desktop.ps1
#Requires -Version 5.1

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$desktop = Join-Path $root 'desktop'

if (-not (Test-Path (Join-Path $desktop 'node_modules\electron'))) {
	Write-Host "正在安装 Electron（首次运行需要联网）..." -ForegroundColor Cyan
	Push-Location $desktop
	try { npm install --no-audit --no-fund } finally { Pop-Location }
}

$backend = Join-Path $root 'x64\Release\RobotServer.exe'
if (-not (Test-Path $backend)) {
	Write-Host "提示：未找到 $backend，Electron 会尝试连接已在运行的后端。" -ForegroundColor Yellow
}

Write-Host "启动 Electron 桌面端..." -ForegroundColor Green
Push-Location $desktop
try { npm start } finally { Pop-Location }
