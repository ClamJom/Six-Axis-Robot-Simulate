//=====================================================================================
// Electron 主进程
//
// 负责：
//   1. 启动 C++ 后端（RobotServer），并等待它监听端口成功
//   2. 打开一个浏览器窗口加载后端提供的前端页面
//   3. 退出时关闭后端进程
//
// 这样桌面端与浏览器访问共用同一套前端代码，后端只写一次。
//=====================================================================================

const { app, BrowserWindow, shell } = require('electron');
const { spawn } = require('node:child_process');
const path = require('node:path');
const http = require('node:http');
const fs = require('node:fs');

const HOST = '127.0.0.1';
const PORT = Number(process.env.ROBOT_PORT || 8760);
const BACKEND_URL = `http://${HOST}:${PORT}/`;

//--no-backend：只开前端窗口，连接已经在运行的后端（后端留给另一个终端启动）
const NO_BACKEND = process.argv.includes('--no-backend') ||
	process.env.ROBOT_NO_BACKEND === '1';

let backend = null;
let mainWindow = null;

/// 找到后端可执行文件：优先用打包目录，其次用 VS 的构建输出
function resolveBackendPath() {
	const candidates = [
		path.join(__dirname, '..', 'bin', 'RobotServer.exe'),
		path.join(__dirname, '..', 'build', 'RobotServer.exe'),
		path.join(__dirname, '..', 'x64', 'Release', 'RobotServer.exe'),
		path.join(__dirname, '..', 'x64', 'Debug', 'RobotServer.exe')
	];
	for (const candidate of candidates) {
		if (fs.existsSync(candidate)) {
			return candidate;
		}
	}
	return null;
}

/// 轮询后端健康检查接口，直到就绪或超时
function waitForBackend(timeoutMs = 20000) {
	const deadline = Date.now() + timeoutMs;
	return new Promise((resolve, reject) => {
		const attempt = () => {
			const request = http.get(
				{ host: HOST, port: PORT, path: '/api/health', timeout: 1000 },
				(response) => {
					response.resume();
					if (response.statusCode === 200) {
						resolve();
					}
					else {
						retry();
					}
				});
			request.on('error', retry);
			request.on('timeout', () => {
				request.destroy();
				retry();
			});
		};
		const retry = () => {
			if (Date.now() > deadline) {
				reject(new Error('后端启动超时'));
				return;
			}
			setTimeout(attempt, 250);
		};
		attempt();
	});
}

function startBackend() {
	const executable = resolveBackendPath();
	if (!executable) {
		console.warn('[electron] 未找到后端可执行文件，请先编译 C++ 工程；');
		console.warn('[electron] 如果后端已在运行，将直接连接它。');
		return null;
	}

	//--web 需要的是「相对后端工作目录」的路径，因此这里传相对路径。
	//绝对路径在 Windows 上会被后端按工作目录再拼一次，导致静态目录不存在、页面全部 404。
	const backendCwd = path.dirname(executable);
	const webRoot = path.relative(backendCwd, path.join(__dirname, '..', 'web'));
	console.log('[electron] 启动后端:', executable);
	console.log('[electron] 工作目录:', backendCwd);
	console.log('[electron] 静态目录(相对):', webRoot);
	const child = spawn(executable, ['--port', String(PORT), '--web', webRoot], {
		cwd: backendCwd,
		stdio: ['ignore', 'pipe', 'pipe'],
		windowsHide: true
	});
	child.stdout.on('data', (data) => process.stdout.write('[backend] ' + data));
	child.stderr.on('data', (data) => process.stderr.write('[backend] ' + data));
	child.on('exit', (code) => {
		console.log('[electron] 后端退出，代码', code);
		backend = null;
	});
	return child;
}

function createWindow() {
	mainWindow = new BrowserWindow({
		width: 1560,
		height: 960,
		minWidth: 1080,
		minHeight: 680,
		backgroundColor: '#0f1116',
		title: '六轴机械臂实时仿真',
		autoHideMenuBar: true,
		webPreferences: {
			contextIsolation: true,
			nodeIntegration: false
		}
	});

	//---------- 加载诊断 ----------
	//页面加载失败时不要只留下一个空白窗口，直接把原因打到终端
	mainWindow.webContents.on('did-fail-load', (event, errorCode, errorDescription, validatedURL) => {
		console.error(`[electron] 页面加载失败：${errorDescription} (${errorCode})`);
		console.error(`[electron] 目标地址：${validatedURL}`);
		console.error('[electron] 请确认后端已启动，且端口与 ROBOT_PORT 一致。');
	});

	mainWindow.webContents.on('console-message', (event, level, message, line, sourceId) => {
		if (level >= 2) {	//2=warning, 3=error
			console.error(`[renderer] ${message} (${sourceId}:${line})`);
		}
	});

	mainWindow.webContents.on('did-finish-load', () => {
		console.log('[electron] 页面加载完成:', mainWindow.webContents.getURL());
	});

	mainWindow.loadURL(BACKEND_URL);
	mainWindow.on('closed', () => { mainWindow = null; });

	//外部链接用系统浏览器打开
	mainWindow.webContents.setWindowOpenHandler(({ url }) => {
		shell.openExternal(url);
		return { action: 'deny' };
	});
}

app.whenReady().then(async () => {
	if (NO_BACKEND) {
		//只开前端：后端由使用者在另一个终端自己启动
		console.log(`[electron] --no-backend：不启动后端，直接连接 ${BACKEND_URL}`);
	}
	else {
		backend = startBackend();
	}
	try {
		await waitForBackend();
	}
	catch (error) {
		console.error('[electron]', error.message);
		if (NO_BACKEND) {
			console.error(`[electron] 请先在另一个终端启动后端，例如：`);
			console.error(`[electron]   x64\\Release\\RobotServer.exe --port ${PORT} --web web`);
		}
	}
	createWindow();

	app.on('activate', () => {
		if (BrowserWindow.getAllWindows().length === 0) {
			createWindow();
		}
	});
});

app.on('window-all-closed', () => {
	if (process.platform !== 'darwin') {
		app.quit();
	}
});

app.on('before-quit', () => {
	if (backend && !backend.killed) {
		console.log('[electron] 关闭后端进程');
		backend.kill();
		backend = null;
	}
});
