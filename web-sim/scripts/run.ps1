# 六轴机械臂仿真 —— 常用脚本
# 用法： pwsh -File scripts\run.ps1
#Requires -Version 5.1

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$port = if ($env:ROBOT_PORT) { $env:ROBOT_PORT } else { '8760' }
$backend = Join-Path $root 'x64\Release\RobotServer.exe'

if (-not (Test-Path $backend)) {
	Write-Host "未找到后端程序：$backend" -ForegroundColor Yellow
	Write-Host "正在用 MSBuild 编译（Release x64）..." -ForegroundColor Cyan
	$msbuild = & "${env:ProgramFiles}\Microsoft Visual Studio\Installer\vswhere.exe" `
		-latest -products * -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' |
		Select-Object -First 1
	if (-not $msbuild) {
		throw "未找到 MSBuild，请先在 Visual Studio 中打开 RobotServer.sln 编译。"
	}
	& $msbuild 'RobotServer.vcxproj' /p:Configuration=Release /p:Platform=x64 /t:Build /v:minimal /nologo /m
	if (-not (Test-Path $backend)) { throw "编译失败" }
}

Write-Host "启动后端：http://127.0.0.1:$port/" -ForegroundColor Green
Write-Host "浏览器打开该地址即可使用；按 Ctrl+C 结束后端。" -ForegroundColor DarkGray
& $backend --port $port --web (Join-Path $root 'web')
