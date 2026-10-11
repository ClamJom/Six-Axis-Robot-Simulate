# 六轴机械臂仿真 —— 端到端验证脚本
# 用法： pwsh -File scripts\verify.ps1
# 会启动后端，用真实 Chromium 加载前端并检查渲染与交互，最后输出截图到 tools\e2e\shots
#Requires -Version 5.1

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$port = 8760
$backend = Join-Path $root 'x64\Release\RobotServer.exe'
if (-not (Test-Path $backend)) { throw "未找到后端程序，请先编译：$backend" }

$e2e = Join-Path $root 'tools\e2e'
if (-not (Test-Path (Join-Path $e2e 'node_modules\playwright'))) {
	Write-Host "正在安装 Playwright（首次运行需要联网）..." -ForegroundColor Cyan
	Push-Location $e2e
	try {
		npm install --no-audit --no-fund
		npx playwright install chromium
	}
	finally { Pop-Location }
}

Write-Host "启动后端（端口 $port）..." -ForegroundColor Cyan
$server = Start-Process -FilePath $backend -WorkingDirectory $root `
	-ArgumentList '--port', $port, '--web', (Join-Path $root 'web') -PassThru
try {
	Start-Sleep -Seconds 3
	Push-Location $e2e
	try {
		$env:ROBOT_URL = "http://127.0.0.1:$port/"
		node verify.js
		$code = $LASTEXITCODE
	}
	finally { Pop-Location }
}
finally {
	if ($server -and -not $server.HasExited) { Stop-Process -Id $server.Id -Force }
}

if ($code -ne 0) { throw "端到端验证失败（退出码 $code）" }
Write-Host "端到端验证通过，截图见 tools\e2e\shots" -ForegroundColor Green
