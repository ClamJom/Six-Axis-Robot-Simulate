//=====================================================================================
// verify.js —— 端到端验证
//
// 在真实 Chromium 中加载前端页面，检查：
//   1. 页面与 Three.js 是否加载成功、有无控制台错误
//   2. 后端状态是否送达（关节角、点云、轨迹）
//   3. WebGL 三维画面是否真的画出了机械臂（检查画布像素颜色分布）
//   4. 交互是否生效（改目标坐标 -> 末端位置随之变化）
//   5. 不可达目标（逆解无解）之后仍然可以继续控制
//   6. 前端对异常状态（JSON null）的容错
//   7. 传送带取件场景：启动后能抓取并码放，暂停后手动控制恢复可用
//=====================================================================================

const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');

const BASE = process.env.ROBOT_URL || 'http://127.0.0.1:8760/';
const OUT = path.join(__dirname, 'shots');

async function main() {
	fs.mkdirSync(OUT, { recursive: true });

	const browser = await chromium.launch({
		args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader']
	});
	const page = await browser.newPage({ viewport: { width: 1560, height: 940 } });

	const errors = [];
	page.on('console', (message) => {
		if (message.type() === 'error') {
			errors.push('[console] ' + message.text());
		}
	});
	page.on('pageerror', (error) => errors.push('[pageerror] ' + error.message));

	console.log('打开页面:', BASE);
	//注意：页面会保持一条 SSE 长连接，因此不能用 networkidle 等待
	await page.goto(BASE, { waitUntil: 'load', timeout: 20000 });

	//等待后端状态到达
	await page.waitForFunction(() => {
		const el = document.getElementById('tcpPos');
		return el && el.textContent && el.textContent.includes('X');
	}, { timeout: 15000 });
	console.log('状态已到达前端');

	//等待 Three.js 完成若干帧渲染
	await page.waitForTimeout(1500);

	//---------- 1. 读取界面上的数值 ----------
	const snapshot = await page.evaluate(() => {
		const text = (id) => {
			const el = document.getElementById(id);
			return el ? el.textContent.trim() : null;
		};
		const value = (id) => {
			const el = document.getElementById(id);
			return el ? el.value : null;
		};
		return {
			tcpPos: text('tcpPos'),
			targetPos: text('targetPos'),
			tcpError: text('tcpError'),
			joints: text('jointDegrees'),
			status: text('status'),
			connection: text('connection'),
			targetX: value('targetX'),
			targetY: value('targetY'),
			targetZ: value('targetZ'),
			canvasCount: document.querySelectorAll('#viewport canvas').length,
			canvasSize: (() => {
				const c = document.querySelector('#viewport canvas');
				return c ? `${c.width}x${c.height}` : null;
			})(),
			jointSliderCount: document.querySelectorAll('#jointSliders input[data-joint]').length
		};
	});
	console.log('界面状态:', JSON.stringify(snapshot, null, 2));

	await page.screenshot({ path: path.join(OUT, '01_initial.png') });

	//---------- 2. 检查 WebGL 画面是否真的画了东西 ----------
	const canvasStats = await page.evaluate(() => {
		const canvas = document.querySelector('#viewport canvas');
		if (!canvas) {
			return { ok: false, reason: '没有画布' };
		}
		//用 2D 画布重新采样，统计非背景像素
		const off = document.createElement('canvas');
		off.width = canvas.width;
		off.height = canvas.height;
		const ctx = off.getContext('2d');
		ctx.drawImage(canvas, 0, 0);
		const data = ctx.getImageData(0, 0, off.width, off.height).data;
		let nonBackground = 0;
		let total = 0;
		const histogram = {};
		for (let i = 0; i < data.length; i += 4 * 37) {	//抽样，够用即可
			total++;
			const r = data[i], g = data[i + 1], b = data[i + 2];
			//背景是 #0b0d12（11,13,18）附近的深色
			if (Math.abs(r - 11) > 14 || Math.abs(g - 13) > 14 || Math.abs(b - 18) > 14) {
				nonBackground++;
			}
			const key = `${r >> 5}-${g >> 5}-${b >> 5}`;
			histogram[key] = (histogram[key] || 0) + 1;
		}
		return {
			ok: nonBackground > total * 0.01,
			sampled: total,
			nonBackground,
			ratio: (nonBackground / total).toFixed(4),
			distinctColors: Object.keys(histogram).length
		};
	});
	console.log('画布像素统计:', JSON.stringify(canvasStats));

	//---------- 3. 交互：修改目标位置并检查末端是否跟随 ----------
	const before = await page.textContent('#tcpPos');
	await page.fill('#targetX', '250');
	await page.fill('#targetY', '-250');
	await page.fill('#targetZ', '500');
	await page.click('#applyPosition');
	await page.waitForTimeout(1200);
	const after = await page.textContent('#tcpPos');
	const targetAfter = await page.textContent('#targetPos');
	const errorAfter = await page.textContent('#tcpError');
	console.log('修改目标前 末端:', before);
	console.log('修改目标后 末端:', after);
	console.log('修改目标后 目标:', targetAfter, ' 误差:', errorAfter);
	await page.screenshot({ path: path.join(OUT, '02_moved.png') });

	//---------- 4. 交互：载入点云包围盒目标 ----------
	await page.click('#loadObb');
	await page.waitForTimeout(1500);
	const afterObb = await page.textContent('#tcpPos');
	const statusAfterObb = await page.textContent('#status');
	console.log('载入包围盒后 末端:', afterObb, ' 状态:', statusAfterObb);
	await page.screenshot({ path: path.join(OUT, '03_obb.png') });

	//---------- 5. 交互：拖动姿态滑块 ----------
	await page.evaluate(() => {
		const slider = document.getElementById('zAxisY');
		slider.value = '0.8';
		slider.dispatchEvent(new Event('input', { bubbles: true }));
	});
	await page.waitForTimeout(1000);
	const afterOrientation = await page.textContent('#tcpPos');
	const zAxisYOut = await page.textContent('#zAxisYOut');
	console.log('调整姿态后 末端:', afterOrientation, ' Z轴Y:', zAxisYOut);
	await page.screenshot({ path: path.join(OUT, '04_orientation.png') });

	//---------- 6. 检查关键区块是否可见 ----------
	const visible = await page.evaluate(() => {
		const ids = ['panel', 'viewport', 'overlay', 'jointSliders'];
		const result = {};
		for (const id of ids) {
			const el = document.getElementById(id);
			const rect = el ? el.getBoundingClientRect() : null;
			result[id] = rect ? `${Math.round(rect.width)}x${Math.round(rect.height)}` : 'missing';
		}
		return result;
	});
	console.log('区块尺寸:', JSON.stringify(visible));

	//---------- 7. 不可达目标之后必须仍然可以继续控制 ----------
	//回归用例：目标不可达时若把 NaN 写回关节，之后任何目标都会继续解出 NaN，
	//状态帧里的目标/末端/关节角会全部变成 null（Json.h 把非有限值写成 null），界面随之失效。
	const readOverlay = () => page.evaluate(() => ({
		tcp: document.getElementById('tcpPos').textContent.trim(),
		target: document.getElementById('targetPos').textContent.trim(),
		error: document.getElementById('tcpError').textContent.trim(),
		status: document.getElementById('status').textContent.trim()
	}));

	const beforeUnreachable = await readOverlay();
	await page.fill('#targetX', '3000');
	await page.fill('#targetY', '3000');
	await page.fill('#targetZ', '3000');
	await page.click('#applyPosition');
	await page.waitForTimeout(1000);
	const unreachable = await readOverlay();
	console.log('不可达目标后 :', JSON.stringify(unreachable));
	await page.screenshot({ path: path.join(OUT, '05_unreachable.png') });

	//随后再给一个可达目标：必须立刻恢复正常
	await page.fill('#targetX', '250');
	await page.fill('#targetY', '-250');
	await page.fill('#targetZ', '500');
	await page.click('#applyPosition');
	await page.waitForTimeout(1000);
	const recovered = await readOverlay();
	console.log('随后可达目标 :', JSON.stringify(recovered));
	await page.screenshot({ path: path.join(OUT, '06_recovered.png') });

	const warned = unreachable.status.includes('逆解无解');
	const heldStill = unreachable.tcp === beforeUnreachable.tcp;		//机械臂保持原位
	const recoveredOk = recovered.tcp.includes('250.0') && parseFloat(recovered.error) < 1;
	const noNulls = [unreachable, recovered].every(
		state => !/null|NaN|--/.test(state.tcp + state.target + state.error));

	//---------- 8. 前端对异常状态（JSON null）的容错 ----------
	//注入一份含 null 的状态：前端不能抛异常（那会中断整帧刷新，界面卡在旧值上再也更新不了）
	const tolerance = await page.evaluate(() => {
		const app = window.robotApp;
		const original = app.state;
		const broken = JSON.parse(JSON.stringify(original));
		broken.model.jointDegrees = broken.model.jointDegrees.map(() => null);
		broken.tcp.position = [null, null, null];
		broken.tcp.error = null;
		broken.target.position = [null, null, null];
		let thrown = null;
		try {
			app.applyState(broken);
		}
		catch (error) {
			thrown = error.message;
		}
		const shown = document.getElementById('tcpPos').textContent.trim();
		app.applyState(original);		//恢复真实状态
		return { thrown, shown };
	});
	console.log('异常状态容错 :', JSON.stringify(tolerance));
	const tolerant = tolerance.thrown === null;

	//---------- 9. 传送带取件场景 ----------
	//先把第 5 节的轨迹采样点拉到 200（两级滑条应当同步），再启动场景
	await page.evaluate(() => {
		const el = document.getElementById('trackSize5');
		el.value = '200';
		el.dispatchEvent(new Event('input', { bubbles: true }));
		el.dispatchEvent(new Event('change', { bubbles: true }));
		const speed = document.getElementById('robotSpeed');
		speed.value = '150';
		speed.dispatchEvent(new Event('input', { bubbles: true }));
		speed.dispatchEvent(new Event('change', { bubbles: true }));
	});
	await page.waitForTimeout(900);
	const knobs = await page.evaluate(() => ({
		trackSize: window.robotApp.state.trajectory.size,
		armTracks: (window.robotApp.state.arms || []).map(a => a.trackSize),
		armSpeeds: (window.robotApp.state.arms || []).map(a => a.speedFactor),
		slider4: document.getElementById('trackSize').value,
		slider5: document.getElementById('trackSize5').value
	}));
	console.log('轨迹点数/臂速:', JSON.stringify(knobs));
	const knobsOk = knobs.trackSize === 200 &&
		knobs.armTracks.every(size => size === 200) &&
		knobs.armSpeeds.every(speed => Math.abs(speed - 1.5) < 1e-6) &&
		knobs.slider4 === '200' && knobs.slider5 === '200';

	//两台臂的抓取工位错开（上游 -300 / 下游 0），保证各自都有活干，检查才确定
	await page.evaluate(() => {
		const rows = document.querySelectorAll('#armList .arm-row');
		const upstream = rows[rows.length - 1];
		upstream.querySelector('[data-field="pickY"]').value = '-300';
		upstream.querySelector('[data-act="pick"]').click();
	});
	await page.waitForTimeout(1000);
	const stations = await page.evaluate(() =>
		(window.robotApp.state.arms || []).map(a => a.pickY));
	console.log('两台臂抓取工位:', JSON.stringify(stations));

	//点击「启动场景」后，后端仿真线程应当驱动机械臂抓取纸箱并码放到收集台上
	await page.click('#startScenario');
	await page.waitForTimeout(2000);
	const scenarioStart = await page.evaluate(() => {
		const view = window.robotApp.view;
		return {
			cellObjects: view.cellGroup.children.length,
			visibleBoxes: view.boxPool.filter(m => m.visible).length,
			gripperVisible: view.gripper.group.visible
		};
	});
	console.log('场景初始:', JSON.stringify(scenarioStart));
	await page.screenshot({ path: path.join(OUT, '07_conveyor.png') });

	//等若干个抓放循环
	await page.waitForTimeout(26000);
	const scenarioState = await page.evaluate(() => {
		const s = window.robotApp.state;
		const arm = (s.arms && s.arms[0]) || {};
		return {
			arms: (s.arms || []).map(a => ({
				id: a.id, name: a.name, ikMode: a.ikMode, phase: a.phase,
				picked: a.picked, stacked: a.stacked, warning: a.warning, pathPoints: (a.path || []).length,
				planMode: a.planMode, planSegments: a.planSegments, planTotal: a.planTotal,
				plannedPoints: (a.planned || []).length
			})),
			arm0: arm,
			visibleBoxes: window.robotApp.view.boxPool.filter(m => m.visible).length,
			rigs: window.robotApp.view.armRigs.length,
			cards: document.querySelectorAll('#armCompare .arm-card').length
		};
	});
	console.log('场景运行中:', scenarioState.arms.map(a =>
		`${a.name} ${a.phase} 抓${a.picked} 码${a.stacked} 轨迹${a.pathPoints} 规划${a.planMode}`).join(' | '));
	console.log('  机械臂数量:', scenarioState.rigs, ' 对比卡片:', scenarioState.cards,
		' 视野内箱子:', scenarioState.visibleBoxes);
	await page.screenshot({ path: path.join(OUT, '08_conveyor_running.png') });

	const scenario = scenarioState.arm0 || {};
	const scenarioOk = scenarioState.arms.some(a => (a.picked || 0) >= 1 && (a.stacked || 0) >= 1) &&
		scenarioState.arms.every(a => !a.warning) && scenarioState.rigs === 2 && scenarioState.cards === 2 &&
		scenarioStart.cellObjects > 10 && scenarioStart.gripperVisible === true;
	//两台臂都要有各自的算法，且都要记录到轨迹；旧结算模式那一台必须走 getTrack 规划
	const twoArmsOk = scenarioState.arms.length === 2 &&
		scenarioState.arms[0].ikMode !== scenarioState.arms[1].ikMode &&
		scenarioState.arms.every(a => a.pathPoints > 0 || a.stacked > 0) &&
		scenarioState.arms.some(a => a.planMode === 'getTrack' && a.planSegments > 0 &&
			a.plannedPoints > 1);

	//---------- 10. 机械臂实体化：每臂独立设置 + 增删臂 + 挪基座 ----------
	//（场景仍在运行，这些操作应当只影响被操作的那一台臂）
	const beforeArm = await page.evaluate(() => ({
		leftTrack: (window.robotApp.state.arms || [])[0].trackSize
	}));
	await page.evaluate(() => {
		const row = document.querySelector('#armList .arm-row:last-child');
		const track = row.querySelector('[data-field="trackSize"]');
		track.value = '80';
		track.dispatchEvent(new Event('change', { bubbles: true }));
		const speed = row.querySelector('[data-field="speed"]');
		speed.value = '250';
		speed.dispatchEvent(new Event('change', { bubbles: true }));
	});
	await page.waitForTimeout(900);
	const perArm = await page.evaluate(() => (window.robotApp.state.arms || []).map(a => ({
		id: a.id, track: a.trackSize, speed: a.speedFactor, base: a.base.map(v => Math.round(v))
	})));
	const perArmOk = perArm.length === 2 && perArm[1].track === 80 &&
		Math.abs(perArm[1].speed - 2.5) < 1e-6 &&
		perArm[0].track === beforeArm.leftTrack && Math.abs(perArm[0].speed - 1.5) < 1e-6;
	console.log('每臂独立设置:', JSON.stringify(perArm), '→', perArmOk);

	//挪动 2 号臂基座，再挪回去
	const movedBase = await page.evaluate(async () => {
		const row = document.querySelector('#armList .arm-row:last-child');
		row.querySelector('[data-field="x"]').value = '1100';
		row.querySelector('[data-field="y"]').value = '-150';
		row.querySelector('[data-act="base"]').click();
		await new Promise(r => setTimeout(r, 1200));
		const arm = (window.robotApp.state.arms || [])[1];
		const ok = Math.abs(arm.base[0] - 1100) < 1 && Math.abs(arm.base[1] + 150) < 1;
		row.querySelector('[data-field="x"]').value = '1200';
		row.querySelector('[data-field="y"]').value = '0';
		row.querySelector('[data-act="base"]').click();
		await new Promise(r => setTimeout(r, 1200));
		return { ok, back: (window.robotApp.state.arms || [])[1].base.map(v => Math.round(v)) };
	});
	console.log('挪动基座:', JSON.stringify(movedBase));
	const baseOk = movedBase.ok && movedBase.back[0] === 1200 && movedBase.back[1] === 0;

	//加一台臂 → 列表/对比卡片/三维视图/场景都要跟上；删掉后模型要隐藏
	await page.evaluate(() => {
		document.getElementById('newArmX').value = '300';
		document.getElementById('newArmY').value = '-600';
		document.getElementById('addArm').click();
	});
	await page.waitForTimeout(1600);
	const added = await page.evaluate(() => ({
		arms: (window.robotApp.state.arms || []).length,
		rows: document.querySelectorAll('#armList .arm-row').length,
		cards: document.querySelectorAll('#armCompare .arm-card').length,
		newest: (window.robotApp.state.arms || [])[2]
	}));
	console.log('添加机械臂:', JSON.stringify({
		arms: added.arms, rows: added.rows, cards: added.cards,
		newId: added.newest && added.newest.id, base: added.newest && added.newest.base
	}));
	const selected = await page.evaluate(async () => {
		document.querySelector('#armList .arm-row:last-child [data-act="select"]').click();
		await new Promise(r => setTimeout(r, 900));
		return {
			manualArm: window.robotApp.state.scenario.manualArm,
			hint: document.getElementById('manualArmHint').textContent.trim()
		};
	});
	console.log('切换手动臂:', JSON.stringify(selected));

	await page.evaluate(() => {
		document.querySelector('#armList .arm-row:last-child [data-act="remove"]').click();
	});
	await page.waitForTimeout(1600);
	const removed = await page.evaluate(() => ({
		arms: (window.robotApp.state.arms || []).length,
		rows: document.querySelectorAll('#armList .arm-row').length,
		visibleRigs: window.robotApp.view.armRigs.filter(r => r.tcpSphere.visible).length,
		manualArm: window.robotApp.state.scenario.manualArm
	}));
	console.log('删除机械臂:', JSON.stringify(removed));
	const armEntityOk = added.arms === 3 && added.rows === 3 && added.cards === 3 &&
		added.newest && added.newest.base[0] === 300 && added.newest.base[1] === -600 &&
		selected.manualArm === added.newest.id && selected.hint.includes('#' + added.newest.id) &&
		removed.arms === 2 && removed.rows === 2 && removed.visibleRigs === 2;
	await page.screenshot({ path: path.join(OUT, '10_arm_entity.png') });

	//---------- 11. 暂停后手动接管 ----------
	//暂停后手动操作应当重新可用
	await page.click('#pauseScenario');
	await page.waitForTimeout(600);
	await page.fill('#targetX', '500');
	await page.fill('#targetY', '300');
	await page.fill('#targetZ', '500');
	await page.click('#applyPosition');
	await page.waitForTimeout(1000);
	const afterPause = await page.textContent('#tcpPos');
	console.log('暂停后手动控制 末端:', afterPause.trim());
	const manualBack = afterPause.includes('500.0') && afterPause.includes('300.0');

	console.log('\n控制台错误数:', errors.length);
	errors.forEach(e => console.log('  ', e));

	await browser.close();

	const moved = before !== after;
	const controllable = warned && heldStill && recoveredOk && noNulls && tolerant;
	const loaded = !errors.length && canvasStats.ok && moved && controllable &&
		snapshot.jointSliderCount === 6 && scenarioOk && manualBack && twoArmsOk && knobsOk &&
		perArmOk && baseOk && armEntityOk;
	console.log('\n=== 结论 ===');
	console.log('  页面加载无错误:', errors.length === 0);
	console.log('  WebGL 有内容 :', canvasStats.ok, `(非背景像素占比 ${canvasStats.ratio})`);
	console.log('  状态已送达   :', !!snapshot.tcpPos);
	console.log('  关节滑块数量 :', snapshot.jointSliderCount);
	console.log('  交互生效     :', moved);
	console.log('  不可达后可控 :', controllable,
		`(提示 ${warned} / 保持原位 ${heldStill} / 恢复 ${recoveredOk} / 无 null ${noNulls} / 容错 ${tolerant})`);
	console.log('  传送带场景   :', scenarioOk,
		scenarioState.arms.map(a => `#${a.id} 抓${a.picked}/码${a.stacked}`).join('　') +
		`　(场景对象 ${scenarioStart.cellObjects})`);
	console.log('  双机械臂     :', twoArmsOk,
		`(${scenarioState.arms.map(a => a.name + '=' + a.ikMode).join(', ')})`);
	console.log('  机械臂实体化 :', perArmOk && baseOk && armEntityOk,
		`(每臂独立设置 ${perArmOk} / 挪基座 ${baseOk} / 增删臂 ${armEntityOk})`);
	console.log('  轨迹点/臂速  :', knobsOk,
		`(轨迹点 ${knobs.trackSize} / 臂速 ${Math.round(knobs.armSpeeds[0] * 100)}%)`);
	console.log('  暂停后手动   :', manualBack);
	console.log('  总体         :', loaded ? '通过' : '失败');
	process.exit(loaded ? 0 : 1);
}

main().catch(error => {
	console.error('验证脚本异常:', error);
	process.exit(1);
});
