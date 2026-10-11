//=====================================================================================
// app.js
//
// 前端应用逻辑：与后端通信并把界面控件接到三维视图上。
//
// 通信方式：
//   * 状态通道：EventSource 订阅 /api/stream（Server-Sent Events），收到完整状态帧
//   * 命令通道：POST /api/command，把用户操作发给后端，响应同样是完整状态帧
//
// 前端不做任何运动学计算，只负责渲染与交互。
//=====================================================================================

import { RobotView } from './RobotView.js';

const $ = (id) => document.getElementById(id);

/// 后端把非有限数值写成 JSON null（见 Json.h）：这里统一格式化。
/// 直接用 null.toFixed() 会抛异常并打断整帧刷新，界面会停在旧值上再也更新不了。
function formatNumber(value, digits, placeholder = '--') {
	return Number.isFinite(value) ? value.toFixed(digits) : placeholder;
}

/// 拖动过程中限制命令发送频率，避免刷爆后端
function throttle(fn, intervalMs) {
	let last = 0;
	let timer = null;
	let pendingArgs = null;
	return function (...args) {
		const now = performance.now();
		if (now - last >= intervalMs) {
			last = now;
			fn.apply(null, args);
			return;
		}
		pendingArgs = args;
		if (timer) {
			return;
		}
		timer = setTimeout(() => {
			timer = null;
			last = performance.now();
			if (pendingArgs) {
				fn.apply(null, pendingArgs);
				pendingArgs = null;
			}
		}, intervalMs - (now - last));
	};
}

class RobotApp {
	constructor() {
		this.view = new RobotView($('viewport'));
		this.state = null;
		this.jointSliderCount = 0;
		this.jointDragging = false;
		this.lastStateAt = 0;
		this.refreshingFromState = false;
		this.display = {
			showToolFrame: true,
			showTargetFrame: true,
			showGrid: true,
			showCloudPoints: true,
			showObb: true,
			showAxes: true
		};

		this.setupCommandQueue();
		this.setupControls();
		this.connect();
		this.animate();
	}

	//-----------------------------------------------------------------------------
	// 与后端通信
	//-----------------------------------------------------------------------------
	setupCommandQueue() {
		this.pendingCommand = null;
		this.sending = false;

		this.sendCommand = (action, payload) => {
			this.pendingCommand = { action, payload: payload || {} };
			this.flushCommands();
		};
	}

	async flushCommands() {
		if (this.sending) {
			return;
		}
		this.sending = true;
		try {
			//命令排队时只保留最新一条：拖滑块时中间过程可以丢弃
			while (this.pendingCommand) {
				const command = this.pendingCommand;
				this.pendingCommand = null;
				try {
					const response = await fetch('api/command', {
						method: 'POST',
						headers: { 'Content-Type': 'application/json' },
						body: JSON.stringify({ action: command.action, ...command.payload })
					});
					if (!response.ok) {
						throw new Error('HTTP ' + response.status);
					}
					const state = await response.json();
					this.applyState(state);
					this.setConnection(true);
				}
				catch (error) {
					this.setConnection(false, error.message);
				}
			}
		}
		finally {
			this.sending = false;
		}
	}

	/// 订阅后端状态推送
	connect() {
		//先取一次当前状态，避免首屏空白
		fetch('api/state')
			.then(r => r.json())
			.then(state => {
				this.applyState(state);
				this.setConnection(true);
			})
			.catch(error => this.setConnection(false, error.message));

		const source = new EventSource('api/stream');
		source.onmessage = (event) => {
			let payload;
			try {
				payload = JSON.parse(event.data);
			}
			catch (error) {
				return;
			}
			if (payload.type === 'ping') {
				this.setConnection(true);
				return;
			}
			if (payload.type === 'state') {
				this.applyState(payload);
				this.setConnection(true);
			}
		};
		source.onerror = () => {
			this.setConnection(false, '状态流中断，正在重连…');
		};
	}

	setConnection(connected, detail) {
		const badge = $('connection');
		badge.textContent = connected ? '后端已连接' : ('未连接' + (detail ? '：' + detail : ''));
		badge.className = 'conn ' + (connected ? 'conn--on' : 'conn--off');
	}

	//-----------------------------------------------------------------------------
	// 状态更新
	//-----------------------------------------------------------------------------
	applyState(state) {
		if (!state || state.type !== 'state') {
			return;
		}
		this.state = state;
		this.lastStateAt = performance.now();

		this.view.showToolFrame = this.display.showToolFrame;
		this.view.showAxes = this.display.showAxes;
		this.view.update(state);

		this.refreshFromState(state);
		this.updateOverlay(state);
	}

	/// 把后端状态回写到控件（用户正在拖动时不打断）
	/// 用一帧完整状态刷新界面
	refreshFromState(state) {
		this.refreshingFromState = true;
		try {
			//目标位置
			if (document.activeElement !== $('targetX')) {
				$('targetX').value = formatNumber(state.target.position[0], 1, '');
			}
			if (document.activeElement !== $('targetY')) {
				$('targetY').value = formatNumber(state.target.position[1], 1, '');
			}
			if (document.activeElement !== $('targetZ')) {
				$('targetZ').value = formatNumber(state.target.position[2], 1, '');
			}

			//目标姿态：由目标矩阵的第三列得到 Z 轴方向
			const m = state.target.matrix;
			const zx = formatNumber(m[2], 3);
			const zy = formatNumber(m[6], 3);
			if (document.activeElement !== $('zAxisX')) {
				$('zAxisX').value = zx;
			}
			if (document.activeElement !== $('zAxisY')) {
				$('zAxisY').value = zy;
			}
			$('zAxisXOut').textContent = zx;
			$('zAxisYOut').textContent = zy;
			//轨迹采样点（第 4 节与第 5 节两个滑块同步）
			['trackSize', 'trackSize5'].forEach(id => {
				if (document.activeElement !== $(id)) {
					$(id).value = state.trajectory.size;
					$(id + 'Out').textContent = state.trajectory.size;
				}
			});

			//关节滑块
			const degrees = state.model.jointDegrees || [];
			this.ensureJointSliders(degrees.length);
			if (!this.jointDragging) {
				for (let i = 0; i < degrees.length; i++) {
					const slider = document.querySelector(`#jointSliders input[data-joint="${i}"]`);
					if (slider) {
						slider.value = formatNumber(degrees[i], 2);
						const out = slider.parentElement.querySelector('output');
						if (out) {
							out.textContent = Number.isFinite(degrees[i])
								? degrees[i].toFixed(1) + '°'
								: '--';
						}
					}
				}
			}

			//按钮的选中态与显示选项保持一致
			$('toggleCloud').classList.toggle('active', !!state.cloud.visible);
			$('toggleTrajectory').classList.toggle('active', !!state.trajectory.visible);

			//目标点云显隐 + 逆解算法（只反映当前选中的那台臂）
			$('toggleTarget').textContent = state.cloud.visible ? '隐藏目标' : '显示目标';
			const legacy = (state.model && state.model.ikMode) === 'legacy';
			const selectedArm = (state.arms || []).find(a => a.selected);
			$('legacyIK').checked = legacy;
			$('manualArmHint').textContent = selectedArm
				? `当前手动操作：#${selectedArm.id} ${selectedArm.name}`
				: '当前手动操作：无（请先在「6. 机械臂管理」里添加/选择）';
			$('ikModeHint').textContent = legacy
				? '当前：旧逆解（单一特解，可达区域有碎片、竖直运动可能翻转 360°）'
				: '当前：完备解析解（最多 8 组，按参考构型就近选解）';

			//---------- 传送带取件场景 ----------
			const scenario = state.scenario || {};
			const capacity = scenario.capacity || 0;
			const arms = state.arms || [];
			$('scenarioStats').textContent =
				`带上 ${scenario.onBelt || 0} 件 · 输送 ${Math.round(scenario.beltSpeed || 0)}mm/s` +
				(scenario.dynamicPick ? ' · 动态抓取' : ' · 定点抓取') +
				(scenario.armCount ? ` · ${scenario.armCount} 台臂` : '') +
				` · 臂速 ${Math.round((scenario.speedFactor || 1) * 100)}%`;

			//机械臂速度滑块（拖动中不要被状态帧回写打断）
			if (document.activeElement !== $('robotSpeed')) {
				const percent = Math.round((scenario.speedFactor || 1) * 100);
				$('robotSpeed').value = percent;
				$('robotSpeedOut').textContent = percent + '%';
			}
			const scenarioMessage = $('scenarioMessage');
			scenarioMessage.textContent = scenario.message || '';
			scenarioMessage.classList.toggle('warn', !!scenario.warning);

			//---------- 各台机械臂的参数对比 ----------
			const selectedId = scenario.manualArm;
			const host = $('armCompare');
			if (host.dataset.count !== String(arms.length)) {
				host.dataset.count = String(arms.length);
				host.innerHTML = '';
				arms.forEach((arm, index) => {
					const card = document.createElement('div');
					card.className = 'arm-card arm-card--' + (index % 2);
					card.innerHTML =
						'<div class="arm-name"></div>' +
						'<div class="arm-line arm-phase"></div>' +
						'<div class="arm-line arm-count"></div>' +
						'<div class="arm-line arm-path"></div>' +
						'<div class="arm-line arm-msg"></div>';
					host.appendChild(card);
				});
			}
			arms.forEach((arm, index) => {
				const card = host.children[index];
				if (!card) {
					return;
				}
				card.querySelector('.arm-name').textContent =
					`#${arm.id} ${arm.name}　[${arm.ikMode === 'legacy' ? '旧特解' : '完备解'}]` +
					(arm.selected ? '　← 手动操作' : '');
				card.querySelector('.arm-phase').textContent =
					`阶段 ${arm.phase}　抓取 ${arm.picked}　码放 ${arm.stacked}/${capacity}` +
					(arm.missed ? `　漏抓 ${arm.missed}` : '') +
					`　臂速 ${Math.round((arm.speedFactor || 1) * 100)}%`;
				card.querySelector('.arm-count').textContent =
					`横向偏移 ${Math.round(arm.lastPickOffsetX || 0)}mm` +
					(scenario.dynamicPick ? `　跟踪偏差 ${(arm.trackError || 0).toFixed(1)}mm` : '');
				card.querySelector('.arm-path').textContent =
					`轨迹 ${(arm.path || []).length} 点　总长 ${Math.round(arm.pathLength || 0)}mm` +
					`　最大跳变 ${(arm.pathMaxJump || 0).toFixed(1)}mm` +
					`　规划 ${arm.planMode === 'getTrack' ? 'getTrack 逐点执行' : '内置插值'}` +
					(arm.planSegments ? `（${arm.planSegments} 段）` : '');
				const msg = card.querySelector('.arm-msg');
				msg.textContent = arm.message || '';
				msg.classList.toggle('warn', !!arm.warning);
			});

			//---------- 机械臂管理面板 ----------
			this.refreshArmList(arms, selectedId);
			$('startScenario').classList.toggle('active', !!scenario.running);
			$('pauseScenario').classList.toggle('active', scenario.running === false);
			$('autoClear').checked = !!scenario.autoClear;
			$('dynamicPick').checked = !!scenario.dynamicPick;
			if (document.activeElement !== $('beltSpeed')) {
				$('beltSpeed').value = scenario.beltSpeed || 160;
				$('beltSpeedOut').textContent = Math.round(scenario.beltSpeed || 160);
			}
			if (document.activeElement !== $('spawnInterval')) {
				const min = scenario.spawnMin || 3;
				$('spawnInterval').value = min;
				$('spawnIntervalOut').textContent = min.toFixed(1) + ' s';
			}

			//状态栏
			const status = state.status || {};
			const footer = $('status');
			footer.textContent = status.message || '就绪';
			footer.classList.toggle('warn', !!status.warning);
		}
		finally {
			this.refreshingFromState = false;
		}
	}

	/// 机械臂管理列表：每台臂一行，可以单独改逆解/速度/轨迹点/基座位置，或选中、删除
	refreshArmList(arms, selectedId) {
		const host = $('armList');
		const signature = arms.map(a => a.id).join(',');
		if (host.dataset.signature !== signature) {
			//机械臂数量变了才重建 DOM；否则只更新内容，避免打断正在拖动的输入框
			host.dataset.signature = signature;
			host.innerHTML = '';
			arms.forEach(() => {
				const row = document.createElement('div');
				row.className = 'arm-row';
				row.innerHTML =
					'<div class="arm-row-head">' +
						'<span class="arm-title"></span>' +
						'<span class="arm-tag"></span>' +
					'</div>' +
					'<div class="row arm-row-buttons">' +
						'<button data-act="select">操作此臂</button>' +
						'<button data-act="remove">删除</button>' +
					'</div>' +
					'<div class="row row--checks">' +
						'<label><input type="checkbox" data-field="legacy" /> 旧逆解（特解）</label>' +
					'</div>' +
					'<div class="row arm-base-row">' +
						'<label>基座 X</label><input type="number" class="num" data-field="x" step="50" />' +
						'<label>Y</label><input type="number" class="num" data-field="y" step="50" />' +
						'<button data-act="base">移动</button>' +
					'</div>' +
					'<div class="row arm-base-row">' +
						'<label>抓取点 Y</label>' +
						'<input type="number" class="num" data-field="pickY" step="50" />' +
						'<button data-act="pick">设定工位</button>' +
					'</div>' +
					'<div class="row arm-param-row">' +
						'<label>速度</label>' +
						'<input type="range" data-field="speed" min="20" max="300" step="10" />' +
						'<output data-out="speed"></output>' +
					'</div>' +
					'<div class="row arm-param-row">' +
						'<label>轨迹点</label>' +
						'<input type="range" data-field="trackSize" min="4" max="200" step="2" />' +
						'<output data-out="trackSize"></output>' +
					'</div>';
				host.appendChild(row);
			});
		}
		arms.forEach((arm, index) => {
			const row = host.children[index];
			if (!row) {
				return;
			}
			row.dataset.arm = String(arm.id);
			row.classList.toggle('selected', arm.id === selectedId);
			row.querySelector('.arm-title').textContent = `#${arm.id} ${arm.name}`;
			row.querySelector('.arm-tag').textContent =
				(arm.id === selectedId ? '手动操作中' : '') +
				(arm.ikMode === 'legacy' ? '　旧特解' : '　完备解') +
				`　工位 y=${Math.round(arm.pickY)}` +
				(arm.claimed || arm.contended
					? `　抢锁 ${arm.claimed} 成/${arm.contended} 拒`
					: '') +
				(arm.lockedBox && arm.lockedBox.length
					? `　锁着 #${arm.lockedBox.join(',#')}`
					: '') +
				(arm.warning ? '　⚠' : '');
			row.querySelector('[data-act="select"]').disabled = (arm.id === selectedId);
			const legacy = row.querySelector('[data-field="legacy"]');
			if (document.activeElement !== legacy) {
				legacy.checked = arm.ikMode === 'legacy';
			}
			const x = row.querySelector('[data-field="x"]');
			const y = row.querySelector('[data-field="y"]');
			if (document.activeElement !== x) {
				x.value = Math.round(arm.base[0]);
			}
			if (document.activeElement !== y) {
				y.value = Math.round(arm.base[1]);
			}
			const pick = row.querySelector('[data-field="pickY"]');
			if (document.activeElement !== pick) {
				pick.value = Math.round(arm.pickY);
			}
			const speed = row.querySelector('[data-field="speed"]');
			if (document.activeElement !== speed) {
				const percent = Math.round((arm.speedFactor || 1) * 100);
				speed.value = percent;
				row.querySelector('[data-out="speed"]').textContent = percent + '%';
			}
			const track = row.querySelector('[data-field="trackSize"]');
			if (document.activeElement !== track) {
				track.value = arm.trackSize || 24;
				row.querySelector('[data-out="trackSize"]').textContent = arm.trackSize || 24;
			}
		});
	}

	/// 机械臂管理面板的事件（事件委托：重建 DOM 后依然有效）
	bindArmListEvents() {
		const host = $('armList');
		const armIdOf = (target) => {
			const row = target.closest ? target.closest('.arm-row') : null;
			return row ? parseInt(row.dataset.arm, 10) : -1;
		};

		host.addEventListener('click', (event) => {
			const action = event.target.dataset ? event.target.dataset.act : null;
			if (!action) {
				return;
			}
			const arm = armIdOf(event.target);
			if (action === 'select') {
				this.sendCommand('selectArm', { arm });
			}
			else if (action === 'remove') {
				this.sendCommand('removeArm', { arm });
			}
			else if (action === 'base') {
				const row = event.target.closest('.arm-row');
				this.sendCommand('setArmBase', {
					arm,
					x: parseFloat(row.querySelector('[data-field="x"]').value) || 0,
					y: parseFloat(row.querySelector('[data-field="y"]').value) || 0
				});
			}
			else if (action === 'pick') {
				//设定这台臂自己的抓取点（传送带上的 Y）
				const row = event.target.closest('.arm-row');
				this.sendCommand('setArmPickY', {
					arm,
					y: parseFloat(row.querySelector('[data-field="pickY"]').value) || 0
				});
			}
		});

		host.addEventListener('change', (event) => {
			const field = event.target.dataset ? event.target.dataset.field : null;
			if (!field) {
				return;
			}
			const arm = armIdOf(event.target);
			if (field === 'legacy') {
				this.sendCommand('setIKMode', {
					arm, mode: event.target.checked ? 'legacy' : 'complete'
				});
			}
			else if (field === 'speed') {
				this.sendCommand('setRobotSpeed', { arm, speed: parseFloat(event.target.value) / 100 });
			}
			else if (field === 'trackSize') {
				this.sendCommand('setTrackSize', { arm, size: parseInt(event.target.value, 10) });
			}
		});

		host.addEventListener('input', (event) => {
			const field = event.target.dataset ? event.target.dataset.field : null;
			const row = event.target.closest ? event.target.closest('.arm-row') : null;
			if (!row || !field) {
				return;
			}
			if (field === 'speed' || field === 'trackSize') {
				const output = row.querySelector(`[data-out="${field}"]`);
				if (output) {
					output.textContent = (field === 'speed')
						? event.target.value + '%'
						: event.target.value;
				}
			}
		});

		$('addArm').addEventListener('click', () => {
			this.sendCommand('addArm', {
				x: parseFloat($('newArmX').value) || 0,
				y: parseFloat($('newArmY').value) || 0
			});
		});
	}

	ensureJointSliders(count) {
		if (this.jointSliderCount === count) {
			return;
		}
		this.jointSliderCount = count;
		const host = $('jointSliders');
		host.innerHTML = '';
		for (let i = 0; i < count; i++) {
			const row = document.createElement('div');
			row.className = 'joint-row';
			const label = document.createElement('span');
			label.textContent = 'J' + (i + 1);
			const slider = document.createElement('input');
			slider.type = 'range';
			slider.min = '-180';
			slider.max = '180';
			slider.step = '0.5';
			slider.value = '0';
			slider.dataset.joint = String(i);
			const out = document.createElement('output');
			out.textContent = '0.0°';
			row.append(label, slider, out);
			host.appendChild(row);
		}
		this.wireJointSliders();
	}

	wireJointSliders() {
		const host = $('jointSliders');
		const emit = () => {
			const degrees = [];
			host.querySelectorAll('input[data-joint]').forEach(slider => {
				degrees.push(parseFloat(slider.value));
			});
			this.sendCommand('setJointAngles', { degrees });
		};
		const throttled = throttle(emit, 70);

		host.querySelectorAll('input[data-joint]').forEach(slider => {
			slider.addEventListener('input', () => {
				const out = slider.parentElement.querySelector('output');
				if (out) {
					out.textContent = parseFloat(slider.value).toFixed(1) + '°';
				}
				throttled();
			});
			slider.addEventListener('pointerdown', () => { this.jointDragging = true; });
			slider.addEventListener('pointerup', () => { this.jointDragging = false; });
			slider.addEventListener('change', () => {
				this.jointDragging = false;
				emit();
			});
		});
	}

	updateOverlay(state) {
		const tcp = state.tcp.position.map(value => formatNumber(value, 1));
		const target = state.target.position.map(value => formatNumber(value, 1));
		const degrees = state.model.jointDegrees || [];
		$('tcpPos').textContent =
			`X ${tcp[0]}  Y ${tcp[1]}  Z ${tcp[2]}`;
		$('targetPos').textContent =
			`X ${target[0]}  Y ${target[1]}  Z ${target[2]}`;
		$('tcpError').textContent = `${formatNumber(state.tcp.error, 3)} mm`;
		$('jointDegrees').textContent =
			degrees.map((d, i) => `J${i + 1} ${formatNumber(d, 1)}°`).join('\n');
	}

	//-----------------------------------------------------------------------------
	// 控件绑定
	//-----------------------------------------------------------------------------
	setupControls() {
		this.bindArmListEvents();
		const sendTargetPosition = () => {
			const position = [
				parseFloat($('targetX').value) || 0,
				parseFloat($('targetY').value) || 0,
				parseFloat($('targetZ').value) || 0
			];
			this.sendCommand('setTargetPosition', { position });
		};
		const sendOrientation = () => {
			this.sendCommand('setOrientation', {
				zAxis: [parseFloat($('zAxisX').value), parseFloat($('zAxisY').value)]
			});
		};

		//1. 目标位置
		$('applyPosition').addEventListener('click', sendTargetPosition);
		['targetX', 'targetY', 'targetZ'].forEach(id => {
			$(id).addEventListener('change', sendTargetPosition);
			$(id).addEventListener('keydown', (event) => {
				if (event.key === 'Enter') {
					sendTargetPosition();
				}
			});
		});
		$('pickFromTarget').addEventListener('click', () => {
			if (this.state) {
				const position = this.state.target.position;
				$('targetX').value = formatNumber(position[0], 1, '');
				$('targetY').value = formatNumber(position[1], 1, '');
				$('targetZ').value = formatNumber(position[2], 1, '');
			}
		});

		//2. 末端姿态
		const orientationThrottled = throttle(sendOrientation, 60);
		['zAxisX', 'zAxisY'].forEach(id => {
			$(id).addEventListener('input', () => {
				$('zAxisXOut').textContent = parseFloat($('zAxisX').value).toFixed(3);
				$('zAxisYOut').textContent = parseFloat($('zAxisY').value).toFixed(3);
				orientationThrottled();
			});
		});

		//3. 位置微调：拖动过程中累加增量，松手后滑块归零
		const setupNudge = (id, axis) => {
			const slider = $(id);
			const out = id + 'Out';
			let accumulated = 0;
			const apply = throttle(() => {
				if (accumulated === 0) {
					return;
				}
				const delta = [0, 0, 0];
				delta[axis] = accumulated;
				accumulated = 0;
				this.sendCommand('nudgePosition', { delta });
			}, 60);

			slider.addEventListener('input', () => {
				accumulated = parseFloat(slider.value);
				$(out).textContent = accumulated.toFixed(0);
				apply();
			});
			const reset = () => {
				slider.value = 0;
				$(out).textContent = '0';
				accumulated = 0;
			};
			slider.addEventListener('change', reset);
			slider.addEventListener('pointerup', reset);
		};
		setupNudge('nudgeX', 0);
		setupNudge('nudgeY', 1);
		setupNudge('nudgeZ', 2);

		//4. 操作
		$('loadObb').addEventListener('click', () => this.sendCommand('loadOBBTarget'));
		$('randomObb').addEventListener('click', () => this.sendCommand('randomOBBTarget'));
		$('toggleTarget').addEventListener('click', () => {
			const visible = !(this.state && this.state.cloud && this.state.cloud.visible);
			this.sendCommand('setTargetVisible', { visible });
		});
		$('goHome').addEventListener('click', () => this.sendCommand('goHome'));
		$('legacyIK').addEventListener('change', () => {
			//只切换当前选中的那台臂（每台臂的逆解互相独立）
			const arm = (this.state && this.state.scenario) ? this.state.scenario.manualArm : -1;
			this.sendCommand('setIKMode', { arm, mode: $('legacyIK').checked ? 'legacy' : 'complete' });
		});
		$('toggleCloud').addEventListener('click', () => this.sendCommand('toggleCloud'));
		$('toggleTrajectory').addEventListener('click', () => {
			const visible = !(this.state && this.state.trajectory.visible);
			this.sendCommand('setTrajectoryVisible', { visible });
		});
		$('toggleAxes').addEventListener('click', (event) => {
			this.display.showAxes = !this.display.showAxes;
			this.view.showAxes = this.display.showAxes;
			event.currentTarget.classList.toggle('active', this.display.showAxes);
			if (this.state) {
				this.view.update(this.state);
			}
		});
		$('focusView').addEventListener('click', () => this.view.focus());

		//5. 轨迹回放：沿后端给出的轨迹点平滑移动目标
		$('replayTrajectory').addEventListener('click', () => this.replayTrajectory());

		//6. 传送带取件场景
		$('startScenario').addEventListener('click', () => this.sendCommand('startScenario'));
		$('pauseScenario').addEventListener('click', () => this.sendCommand('pauseScenario'));
		$('resetScenario').addEventListener('click', () => this.sendCommand('resetScenario'));
		$('beltSpeed').addEventListener('input', () => {
			$('beltSpeedOut').textContent = $('beltSpeed').value;
		});
		$('beltSpeed').addEventListener('change', () => {
			this.sendCommand('setBeltSpeed', { speed: parseFloat($('beltSpeed').value) });
		});
		$('spawnInterval').addEventListener('input', () => {
			$('spawnIntervalOut').textContent = parseFloat($('spawnInterval').value).toFixed(1) + ' s';
		});
		$('spawnInterval').addEventListener('change', () => {
			//滑块给的是最短间隔，最长间隔取「最短 + 2.5s」，保持随机性
			const min = parseFloat($('spawnInterval').value);
			this.sendCommand('setSpawnInterval', { min, max: min + 2.5 });
		});
		$('autoClear').addEventListener('change', () => {
			this.sendCommand('setAutoClear', { enabled: $('autoClear').checked });
		});
		$('dynamicPick').addEventListener('change', () => {
			this.sendCommand('setDynamicPick', { enabled: $('dynamicPick').checked });
		});
		$('robotSpeed').addEventListener('input', () => {
			$('robotSpeedOut').textContent = $('robotSpeed').value + '%';
		});
		$('robotSpeed').addEventListener('change', () => {
			this.sendCommand('setRobotSpeed', { speed: parseFloat($('robotSpeed').value) / 100 });
		});
		//场景里的轨迹采样点与第 4 节那个是同一个量，两处滑块互相同步
		const trackSliders = ['trackSize', 'trackSize5'];
		trackSliders.forEach(id => {
			$(id).addEventListener('input', () => {
				$(id + 'Out').textContent = $(id).value;
			});
			$(id).addEventListener('change', () => {
				const size = parseInt($(id).value, 10);
				trackSliders.forEach(other => {
					$(other).value = size;
					$(other + 'Out').textContent = size;
				});
				this.sendCommand('setTrackSize', { size });
			});
		});

		//7. 显示选项（纯前端开关）
		const bindToggle = (id, key, apply) => {
			$(id).addEventListener('change', () => {
				this.display[key] = $(id).checked;
				if (apply) {
					apply();
				}
				if (this.state) {
					this.view.update(this.state);
				}
			});
		};
		bindToggle('showTool', 'showToolFrame');
		bindToggle('showTarget', 'showTargetFrame');
		bindToggle('showGrid', 'showGrid', () => { this.view.grid.visible = this.view.showGrid; });
		bindToggle('showCloudPoints', 'showCloudPoints');
		bindToggle('showObb', 'showObb');

		$('toggleAxes').classList.add('active');
		$('toggleCloud').classList.add('active');
		$('toggleTrajectory').classList.add('active');
		this.view.grid.visible = this.display.showGrid;
	}

	/// 沿轨迹点回放：逐个把目标位置发给后端，形成连续动画
	replayTrajectory() {
		if (!this.state || !this.state.trajectory.points || this.state.trajectory.points.length < 2) {
			return;
		}
		if (this.replaying) {
			return;
		}
		const points = this.state.trajectory.points.slice().reverse();
		this.replaying = true;
		let index = 0;
		const step = () => {
			if (index >= points.length) {
				this.replaying = false;
				return;
			}
			const p = points[index++];
			this.sendCommand('setTargetPosition', { position: [p[0], p[1], p[2]] });
			setTimeout(step, 60);
		};
		step();
	}

	//-----------------------------------------------------------------------------
	// 渲染循环
	//-----------------------------------------------------------------------------
	animate() {
		const loop = () => {
			requestAnimationFrame(loop);
			this.view.render();
		};
		loop();
	}
}

/// 移除启动提示块（「正在加载」遮罩，以及模块加载失败时的排查指引）
function clearBootOverlays() {
	window.__robotAppBooted = true;
	document.querySelectorAll('[data-app-block]').forEach((element) => {
		element.remove();
	});
}

/// 在遮罩上显示错误信息；若遮罩已被移除则仅记录到控制台
function showBootError(message) {
	const loading = document.getElementById('loading');
	if (loading) {
		loading.textContent = message;
		loading.classList.remove('hidden');
	}
	console.error(message);
}

//启动应用
window.addEventListener('DOMContentLoaded', () => {
	try {
		clearBootOverlays();
		//实例挂到 window 上便于调试，端到端测试也靠它读取状态
		window.robotApp = new RobotApp();
	}
	catch (error) {
		showBootError('初始化失败：' + error.message);
		console.error(error);
	}
});
