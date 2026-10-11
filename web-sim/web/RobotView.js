//=====================================================================================
// RobotView.js
//
// 三维渲染模块：只负责把后端推送的状态画出来（Three.js / WebGL）。
// 不包含任何运动学计算——所有数值都来自后端。
//
// 后端坐标约定：Z 轴竖直向上，单位为毫米（mm）。
//=====================================================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const TOOL_COLOR = 0xffffff;
const TARGET_COLOR = 0xff8c1a;
const TRACK_COLOR = 0x4be07a;
const TCP_COLOR = 0x33ff88;

/// 每台机械臂一个主色（按顺序循环），便于在视图里区分
const ARM_COLORS = [0x66ccff, 0xffa64d, 0x9be36a, 0xff7ad1, 0xc9a0ff, 0xffe066];

function colorForArm(arm, isFirst) {
	if (typeof arm.id === 'number' && arm.id > 0) {
		return ARM_COLORS[(arm.id - 1) % ARM_COLORS.length];
	}
	return isFirst ? ARM_COLORS[0] : ARM_COLORS[1];
}

const AXIS_COLORS = [0xff4d4d, 0x4dff5a, 0x598cff];	//X / Y / Z

const _vecA = new THREE.Vector3();
const _vecB = new THREE.Vector3();
const _dir = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _mat = new THREE.Matrix4();
const _color = new THREE.Color();

const UP = new THREE.Vector3(0, 0, 1);

/// 传送带带面的条纹贴图（靠滚动 offset 表现皮带运动）
function makeBeltTexture() {	const canvas = document.createElement('canvas');
	canvas.width = 8;
	canvas.height = 64;
	const ctx = canvas.getContext('2d');
	ctx.fillStyle = '#23272e';
	ctx.fillRect(0, 0, 8, 64);
	ctx.fillStyle = '#333944';
	ctx.fillRect(0, 0, 8, 14);
	const texture = new THREE.CanvasTexture(canvas);
	texture.wrapS = THREE.RepeatWrapping;
	texture.wrapT = THREE.RepeatWrapping;
	return texture;
}

/// 两指夹爪：
///   局部原点 = 工具末端（TCP）= 抓取点（箱子上表面）
///   局部 -Z 方向指向法兰（工具长 119mm），夹爪就装在那块法兰上
///   局部 +Z 方向背离法兰，也就是「往下罩住箱子」的方向，手指从这里伸出去
function makeGripper(boxHeight) {
	const group = new THREE.Group();
	group.visible = false;

	const metal = new THREE.MeshStandardMaterial({ color: 0x8d99a8, roughness: 0.4, metalness: 0.75 });
	const fingerMaterial = new THREE.MeshStandardMaterial({ color: 0xd8dee8, roughness: 0.28, metalness: 0.6 });
	const TOOL_LENGTH = 119;			//与 RobotUtils::createDefaultRobot 里的工具 d 一致

	//法兰盘（贴着机器人第 6 轴安装面）
	const flange = new THREE.Mesh(new THREE.CylinderGeometry(33, 33, 12, 20), metal);
	flange.rotation.x = Math.PI / 2;
	flange.position.set(0, 0, -TOOL_LENGTH + 6);
	group.add(flange);

	//夹爪本体（法兰 → 手掌之间），半径比腕部关节球小，避免糊住 4/5/6 轴
	const body = new THREE.Mesh(new THREE.CylinderGeometry(22, 25, 74, 18), metal);
	body.rotation.x = Math.PI / 2;
	body.position.set(0, 0, -TOOL_LENGTH + 12 + 37);
	group.add(body);

	//手掌：压在箱子上表面稍上方
	const palm = new THREE.Mesh(new THREE.BoxGeometry(86, 62, 26), metal);
	palm.position.set(0, 0, -26);
	group.add(palm);

	//两根手指：从手掌向下包住箱子侧面，长度取箱高的 3/4（指尖不戳到台面/带面）
	const fingerLength = Math.max(40, boxHeight * 0.75);
	const fingers = [];
	for (const side of [-1, 1]) {
		const finger = new THREE.Mesh(new THREE.BoxGeometry(13, 26, fingerLength), fingerMaterial);
		finger.position.set(side * 60, 0, fingerLength * 0.5 - 8);
		group.add(finger);
		fingers.push(finger);
	}
	return { group, fingers, fingerLength };
}

/// 把后端下发的行主序 4x4 矩阵写入 Three.js 矩阵
function applyMatrix(target, values) {
	if (!values || values.length < 16) {
		target.identity();
		return target;
	}
	//Three.js 的 set() 接收行主序参数，与后端一致
	target.set(
		values[0], values[1], values[2], values[3],
		values[4], values[5], values[6], values[7],
		values[8], values[9], values[10], values[11],
		values[12], values[13], values[14], values[15]);
	return target;
}

/// 把长度为 n 的数组按需扩容/复用
function ensurePool(pool, count, factory) {
	while (pool.length < count) {
		pool.push(factory(pool.length));
	}
	for (let i = 0; i < pool.length; i++) {
		pool[i].visible = i < count;
	}
	return pool;
}

export class RobotView {
	constructor(container) {
		this.container = container;

		this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
		this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
		this.renderer.setSize(container.clientWidth, container.clientHeight);
		this.renderer.setClearColor(0x0b0d12, 1);
		container.appendChild(this.renderer.domElement);

		this.scene = new THREE.Scene();
		this.scene.fog = new THREE.Fog(0x0b0d12, 3000, 9000);

		this.camera = new THREE.PerspectiveCamera(
			38, container.clientWidth / container.clientHeight, 1, 40000);
		this.camera.up.copy(UP);
		this.camera.position.set(1150, -2750, 1950);

		this.controls = new OrbitControls(this.camera, this.renderer.domElement);
		this.controls.enableDamping = true;
		this.controls.dampingFactor = 0.08;
		this.controls.target.set(620, 0, 280);
		this.controls.minDistance = 200;
		this.controls.maxDistance = 8000;
		this.controls.update();

		//---------- 光照 ----------
		this.scene.add(new THREE.HemisphereLight(0xbcd4ff, 0x20242c, 1.15));
		const keyLight = new THREE.DirectionalLight(0xffffff, 1.15);
		keyLight.position.set(1200, -900, 1800);
		this.scene.add(keyLight);
		const fillLight = new THREE.DirectionalLight(0x88aaff, 0.45);
		fillLight.position.set(-1400, 1000, 600);
		this.scene.add(fillLight);

		//---------- 地面网格（XY 平面，Z=0） ----------
		this.grid = new THREE.GridHelper(4000, 40, 0x2a3140, 0x1c2129);
		this.grid.rotation.x = Math.PI / 2;		//GridHelper 默认在 XZ 平面，这里转到 XY
		this.grid.position.set(0, 0, 0);
		this.scene.add(this.grid);

		//世界坐标轴
		this.worldAxes = new THREE.AxesHelper(320);
		this.worldAxes.position.set(0, 0, 0);
		this.scene.add(this.worldAxes);

		//---------- 几何对象池 ----------
		this.root = new THREE.Group();
		this.scene.add(this.root);

		this.linkPool = [];
		this.jointPool = [];
		this.axisPool = [];
		this.state = null;
		this.showAxes = true;

		//---------- 目标坐标系 ----------
		this.targetGroup = new THREE.Group();
		this.targetFrame = this.makeFrame(160, TARGET_COLOR);
		this.targetGroup.add(this.targetFrame.group);
		this.targetSphere = new THREE.Mesh(
			new THREE.SphereGeometry(9, 16, 12),
			new THREE.MeshStandardMaterial({ color: TARGET_COLOR, emissive: TARGET_COLOR, emissiveIntensity: 0.4 }));
		this.targetGroup.add(this.targetSphere);
		this.root.add(this.targetGroup);

		//---------- 轨迹 ----------
		this.trackMaterial = new THREE.LineBasicMaterial({ color: TRACK_COLOR });
		this.trackLine = new THREE.Line(new THREE.BufferGeometry(), this.trackMaterial);
		this.trackLine.frustumCulled = false;
		this.root.add(this.trackLine);

		//---------- 点云 ----------
		//关闭 sizeAttenuation：工作空间有近 1 m 跨度，若按透视缩放，远处点会小到看不见
		this.cloudMaterial = new THREE.PointsMaterial({
			size: 3.2, sizeAttenuation: false, vertexColors: true, transparent: true, opacity: 0.95
		});
		this.cloudPoints = new THREE.Points(new THREE.BufferGeometry(), this.cloudMaterial);
		this.cloudPoints.frustumCulled = false;
		this.cloudPoints.renderOrder = 1;
		this.root.add(this.cloudPoints);

		//---------- 包围盒 ----------
		this.obbBox = new THREE.LineSegments(
			new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
			new THREE.LineBasicMaterial({ color: 0x4fd1ff, transparent: true, opacity: 0.9 }));
		this.obbBox.frustumCulled = false;
		this.obbBox.visible = false;
		this.root.add(this.obbBox);

		//---------- 工具末端标记（第一台臂，兼容旧代码）----------
		this.tcpSphere = new THREE.Mesh(
			new THREE.SphereGeometry(7, 16, 12),
			new THREE.MeshStandardMaterial({ color: TCP_COLOR, emissive: TCP_COLOR, emissiveIntensity: 0.5 }));
		this.root.add(this.tcpSphere);

		//---------- 传送带取件场景：传送带/收集台 + 货物 + 夹爪 ----------
		this.cellGroup = new THREE.Group();		//静态场景（布局不变时不重建）
		this.root.add(this.cellGroup);
		this.cellKey = '';
		this.beltTexture = makeBeltTexture();
		this.beltSpeed = 0;
		this.beltLength = 3000;

		this.boxGroup = new THREE.Group();
		this.root.add(this.boxGroup);
		this.boxPool = [];

		//---------- 多台机械臂：每台一套连杆/关节/末梢/夹爪/轨迹线 ----------
		this.armRigs = [];
		this.armGroup = new THREE.Group();
		this.root.add(this.armGroup);
		this.gripper = null;		//= 第一台臂的夹爪（拿到布局后按箱高构建）

		this.demoHidden = false;	//场景运行时隐藏点云/包围盒/目标坐标系
		this.clock = new THREE.Clock();

		this.showToolFrame = true;
		this.showTargetFrame = true;
		this.showGrid = true;
		this.showCloudPoints = true;
		this.showObb = true;

		window.addEventListener('resize', () => this.resize());
		this.resize();
	}

	/// 生成一个由三条线段组成的坐标系对象
	makeFrame(scale, color) {
		const group = new THREE.Group();
		const lines = [];
		for (let axis = 0; axis < 3; axis++) {
			const geometry = new THREE.BufferGeometry().setFromPoints([
				new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, 0)
			]);
			const line = new THREE.Line(geometry,
				new THREE.LineBasicMaterial({ color: AXIS_COLORS[axis] }));
			line.frustumCulled = false;
			lines.push(line);
			group.add(line);
		}
		return { group, lines, scale, color };
	}

	/// 用矩阵设置坐标系的三条轴线（矩阵的列即 X/Y/Z 基向量）
	updateFrame(frame, matrix) {
		const origin = new THREE.Vector3().setFromMatrixPosition(matrix);
		const basis = [
			new THREE.Vector3(matrix.elements[0], matrix.elements[1], matrix.elements[2]),
			new THREE.Vector3(matrix.elements[4], matrix.elements[5], matrix.elements[6]),
			new THREE.Vector3(matrix.elements[8], matrix.elements[9], matrix.elements[10])
		];
		for (let axis = 0; axis < 3; axis++) {
			const end = basis[axis].clone().normalize().multiplyScalar(frame.scale).add(origin);
			const positions = frame.lines[axis].geometry.attributes.position;
			positions.setXYZ(0, origin.x, origin.y, origin.z);
			positions.setXYZ(1, end.x, end.y, end.z);
			positions.needsUpdate = true;
			frame.lines[axis].geometry.computeBoundingSphere();
		}
	}


	/// 更新目标坐标系与目标点
	updateTarget(target) {
		this.targetGroup.visible = this.showTargetFrame && !this.demoHidden;
		if (!this.targetGroup.visible || !target) {
			return;
		}
		applyMatrix(_mat, target.matrix);
		this.updateFrame(this.targetFrame, _mat);
		this.targetSphere.position.setFromMatrixPosition(_mat);
	}

	/// 更新轨迹折线
	updateTrajectory(trajectory) {
		const visible = !this.demoHidden && trajectory && trajectory.visible &&
			trajectory.points && trajectory.points.length > 1;
		this.trackLine.visible = visible;
		if (!visible) {
			return;
		}
		const points = trajectory.points.map(p => new THREE.Vector3(p[0], p[1], p[2]));
		this.trackLine.geometry.dispose();
		this.trackLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
		this.trackLine.geometry.computeBoundingSphere();
	}

	/// 更新点云（按位置着色，颜色在后端给定的范围内渐变）
	updateCloud(cloud) {
		const visible = !this.demoHidden && this.showCloudPoints && cloud && cloud.visible &&
			cloud.points && cloud.points.length > 0;
		this.cloudPoints.visible = visible;
		if (!visible) {
			return;
		}
		//点云很少变化：版本号没变就不重建几何（场景运行时每帧都会推状态帧）
		if (cloud.revision !== undefined && cloud.revision === this.cloudRevision) {
			return;
		}
		this.cloudRevision = cloud.revision;
		const count = cloud.points.length;
		const positions = new Float32Array(count * 3);
		const colors = new Float32Array(count * 3);
		//按 Z 高度做冷暖渐变，便于分辨层次
		let minZ = Infinity, maxZ = -Infinity;
		for (const p of cloud.points) {
			if (p[2] < minZ) minZ = p[2];
			if (p[2] > maxZ) maxZ = p[2];
		}
		const span = Math.max(1e-3, maxZ - minZ);
		for (let i = 0; i < count; i++) {
			const p = cloud.points[i];
			positions[i * 3] = p[0];
			positions[i * 3 + 1] = p[1];
			positions[i * 3 + 2] = p[2];
			const t = (p[2] - minZ) / span;
			_color.setHSL(0.58 - 0.58 * t, 0.75, 0.35 + 0.3 * t);
			colors[i * 3] = _color.r;
			colors[i * 3 + 1] = _color.g;
			colors[i * 3 + 2] = _color.b;
		}
		this.cloudPoints.geometry.dispose();
		const geometry = new THREE.BufferGeometry();
		geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
		geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
		geometry.computeBoundingSphere();
		this.cloudPoints.geometry = geometry;
	}

	//-----------------------------------------------------------------------------
	// 传送带取件场景
	//-----------------------------------------------------------------------------

	/// 按后端给的布局搭建（或重建）传送带与收集台
	buildCell(layout, arms) {
		if (!layout) {
			return;
		}
		const tableKey = (arms || []).map(a => (a.slots || []).flat().join(',')).join('|') +
			'#' + (arms || []).map(a => a.pickY).join(',');
		const key = JSON.stringify(layout) + '#' + tableKey;
		if (key === this.cellKey) {
			return;
		}
		this.cellKey = key;

		//夹爪尺寸与箱高相关，布局变化时重建
		if (this.gripper) {
			this.root.remove(this.gripper.group);
			this.gripper = null;
		}
		this.gripper = makeGripper(layout.boxHeight || 100);
		this.root.add(this.gripper.group);

		//清空旧场景
		for (const child of this.cellGroup.children.slice()) {
			this.cellGroup.remove(child);
			if (child.geometry) {
				child.geometry.dispose();
			}
		}

		const beltLength = Math.max(1, layout.beltEndY - layout.beltBeginY);
		const beltCenterY = (layout.beltEndY + layout.beltBeginY) * 0.5;
		const beltWidth = layout.beltHalfWidth * 2;
		const beltTopZ = layout.beltTopZ;
		this.beltLength = beltLength;

		//带面：平面 + 条纹贴图，法线朝 +Z（本场景 Z 轴向上）
		this.beltTexture.repeat.set(1, Math.max(1, Math.round(beltLength / 90)));
		const beltTop = new THREE.Mesh(
			new THREE.PlaneGeometry(beltWidth, beltLength),
			new THREE.MeshStandardMaterial({ map: this.beltTexture, roughness: 0.92, metalness: 0.05 }));
		beltTop.position.set(layout.beltX, beltCenterY, beltTopZ);
		this.cellGroup.add(beltTop);

		//带身
		const body = new THREE.Mesh(
			new THREE.BoxGeometry(beltWidth, beltLength, 60),
			new THREE.MeshStandardMaterial({ color: 0x2c313a, roughness: 0.85, metalness: 0.2 }));
		body.position.set(layout.beltX, beltCenterY, beltTopZ - 32);
		this.cellGroup.add(body);

		//护栏与支腿
		const railMaterial = new THREE.MeshStandardMaterial({ color: 0x59636f, roughness: 0.5, metalness: 0.5 });
		const legMaterial = new THREE.MeshStandardMaterial({ color: 0x3a4048, roughness: 0.7, metalness: 0.3 });
		for (const side of [-1, 1]) {
			const rail = new THREE.Mesh(new THREE.BoxGeometry(14, beltLength, 70), railMaterial);
			rail.position.set(layout.beltX + side * (layout.beltHalfWidth + 7), beltCenterY, beltTopZ + 15);
			this.cellGroup.add(rail);
			for (let y = layout.beltBeginY + 260; y < layout.beltEndY; y += 560) {
				const leg = new THREE.Mesh(
					new THREE.BoxGeometry(36, 36, Math.max(40, beltTopZ - 30)), legMaterial);
				leg.position.set(layout.beltX + side * (layout.beltHalfWidth - 26), y,
					Math.max(40, beltTopZ - 30) * 0.5);
				this.cellGroup.add(leg);
			}
		}

		//抓取工位标记（挡停位置）
		const mark = new THREE.Mesh(
			new THREE.BoxGeometry(beltWidth, 12, 6),
			new THREE.MeshStandardMaterial({ color: 0xffc247, emissive: 0xffa000, emissiveIntensity: 0.7 }));
		mark.position.set(layout.beltX, layout.pickY, beltTopZ + 4);
		this.cellGroup.add(mark);

		//每台臂自己的抓取工位：横跨带面的一条色线 + 一块小牌，颜色与该臂一致
		const armAccent = [0x66ccff, 0xffa64d, 0x9be36a, 0xff7ad1, 0xc9a0ff, 0xffe066];
		(arms || []).forEach((arm, index) => {
			const color = armAccent[(Math.max(1, arm.id) - 1) % armAccent.length];
			const station = new THREE.Mesh(
				new THREE.BoxGeometry(beltWidth, 10, 5),
				new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.55 }));
			station.position.set(layout.beltX, arm.pickY, beltTopZ + 3);
			this.cellGroup.add(station);
			//带面外侧的立牌，标出这是哪台臂的工位
			const plate = new THREE.Mesh(
				new THREE.BoxGeometry(10, 46, 120),
				new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.25 }));
			plate.position.set(layout.beltX - layout.beltHalfWidth - 26, arm.pickY, beltTopZ + 60);
			this.cellGroup.add(plate);
		});

		//收集台：**每台机械臂一个**（台面 + 台腿 + 垛位角标），用各自的主色区分
		const armList = arms || [];
		const accent = [0x4fd1ff, 0xffa64d, 0x9be36a, 0xff7ad1];
		for (let index = 0; index < armList.length; index++) {
			const slots = armList[index].slots || [];
			if (slots.length === 0) {
				continue;
			}
			const xs = slots.map(s => s[0]);
			const ys = slots.map(s => s[1]);
			const tableWidth = Math.max(240, Math.max(...xs) - Math.min(...xs) + 300);
			const tableLength = Math.max(300, Math.max(...ys) - Math.min(...ys) + 320);
			const centerX = (Math.min(...xs) + Math.max(...xs)) * 0.5;
			const centerY = (Math.min(...ys) + Math.max(...ys)) * 0.5;

			const table = new THREE.Mesh(
				new THREE.BoxGeometry(tableWidth, tableLength, 30),
				new THREE.MeshStandardMaterial({ color: 0x6d5b45, roughness: 0.88, metalness: 0.04 }));
			table.position.set(centerX, centerY, layout.palletTopZ - 15);
			this.cellGroup.add(table);

			for (const side of [-1, 1]) {
				for (const end of [-1, 1]) {
					const leg = new THREE.Mesh(new THREE.BoxGeometry(40, 40, Math.max(40, layout.palletTopZ - 30)),
						legMaterial);
					leg.position.set(centerX + side * (tableWidth * 0.5 - 40),
						centerY + end * (tableLength * 0.5 - 40),
						Math.max(40, layout.palletTopZ - 30) * 0.5);
					this.cellGroup.add(leg);
				}
			}

			const markerSize = (layout.boxMaxFootprint || 180) + 24;
			for (const slot of slots) {
				const marker = new THREE.Mesh(
					new THREE.BoxGeometry(markerSize, markerSize, 3),
					new THREE.MeshBasicMaterial({
						color: accent[index % accent.length], transparent: true, opacity: 0.22 }));
				marker.position.set(slot[0], slot[1], layout.palletTopZ + 2);
				this.cellGroup.add(marker);
			}
		}

		console.log('[view] 传送带场景已构建：带长 %s mm，%d 台机械臂的收集台',
			Math.round(beltLength), armList.length);
	}

	/// 更新所有货物箱（传送带上的、夹持中的、已码放的）
	updateBoxes(boxes) {
		const list = boxes || [];
		while (this.boxPool.length < list.length) {
			const mesh = new THREE.Mesh(
				new THREE.BoxGeometry(1, 1, 1),
				new THREE.MeshStandardMaterial({ color: 0xc8a06a, roughness: 0.78, metalness: 0.05 }));
			this.boxGroup.add(mesh);
			this.boxPool.push(mesh);
		}
		for (let i = 0; i < this.boxPool.length; i++) {
			const mesh = this.boxPool[i];
			const box = list[i];
			if (!box) {
				mesh.visible = false;
				continue;
			}
			mesh.visible = true;
			mesh.position.set(box.p[0], box.p[1], box.p[2]);
			mesh.scale.set(box.s[0], box.s[1], box.s[2]);
			mesh.rotation.set(0, 0, box.yaw || 0);
			mesh.material.color.setRGB(box.c[0], box.c[1], box.c[2]);
			//被夹持的箱子加一点自发光，方便一眼看出「抓住了」
			const held = box.state === 'held';
			mesh.material.emissive.setHex(held ? 0x5a4318 : 0x000000);
			mesh.material.emissiveIntensity = held ? 0.35 : 0;
		}
	}


	//-----------------------------------------------------------------------------
	// 多台机械臂
	//-----------------------------------------------------------------------------

	/// 取得（或创建）第 index 台臂的绘制装置
	armRig(index, color) {
		while (this.armRigs.length <= index) {
			const id = this.armRigs.length;
			const rig = {
				id,
				linkPool: [],
				jointPool: [],
				axisPool: [],
				baseMarker: null,
				tcpSphere: new THREE.Mesh(
					new THREE.SphereGeometry(7, 16, 12),
					new THREE.MeshStandardMaterial({
						color: TCP_COLOR, emissive: TCP_COLOR, emissiveIntensity: 0.5 })),
				pathLine: new THREE.Line(
					new THREE.BufferGeometry(),
					new THREE.LineBasicMaterial({ color: color || 0x66ccff, transparent: true, opacity: 0.95 })),
				planLine: new THREE.Line(
					new THREE.BufferGeometry(),
					new THREE.LineDashedMaterial({
						color: color || 0x66ccff, transparent: true, opacity: 0.8,
						dashSize: 30, gapSize: 18 })),
				pathRevision: -1,
				planRevision: -1,
				gripper: null
			};
			rig.pathLine.frustumCulled = false;
			rig.planLine.frustumCulled = false;
			this.armGroup.add(rig.tcpSphere);
			this.armGroup.add(rig.pathLine);
			this.armGroup.add(rig.planLine);
			this.armRigs.push(rig);
		}
		return this.armRigs[index];
	}

	/// 更新一台机械臂的连杆/关节/坐标系/末梢/夹爪/轨迹
	updateArmRig(arm, layout, isFirst, index) {
		//按「在列表里的位置」分配绘制装置，而不是按臂编号（编号可以随便删/加）
		const rig = this.armRig((index === undefined) ? arm.id : index,
			colorForArm(arm, isFirst));
		rig.hidden = false;
		rig.tcpSphere.visible = true;
		const model = arm.model;
		const tcpMatrix = arm.tcp && arm.tcp.matrix;
		this.updateRobotRig(rig, model, tcpMatrix, isFirst);

		//夹爪（按箱高构建）
		if (!rig.gripper && layout) {
			rig.gripper = makeGripper(layout.boxHeight || 100);
			this.armGroup.add(rig.gripper.group);
		}
		if (rig.gripper) {
			const opening = Math.max(24, arm.gripper || 240);
			for (let i = 0; i < rig.gripper.fingers.length; i++) {
				const side = (i === 0) ? -1 : 1;
				rig.gripper.fingers[i].position.x = side * (opening * 0.5 + 7);
			}
			this.updateGripper_(rig, arm, tcpMatrix);
		}

		//轨迹线：实线 = 实际走出来的（每帧采样的 TCP），虚线 = getTrack 规划出来的这一段
		const points = arm.path || [];
		if (arm.pathRevision !== rig.pathRevision) {
			rig.pathRevision = arm.pathRevision;
			rig.pathLine.geometry.dispose();
			const geometry = new THREE.BufferGeometry().setFromPoints(
				points.map(p => new THREE.Vector3(p[0], p[1], p[2])));
			geometry.computeBoundingSphere();
			rig.pathLine.geometry = geometry;
		}
		rig.pathLine.visible = points.length > 1;

		const planned = arm.planned || [];
		const planKey = (arm.planSegments || 0) + ':' + planned.length + ':' + (arm.planMode || '');
		if (planKey !== rig.planRevision) {
			rig.planRevision = planKey;
			rig.planLine.geometry.dispose();
			const geometry = new THREE.BufferGeometry().setFromPoints(
				planned.map(p => new THREE.Vector3(p[0], p[1], p[2])));
			geometry.computeBoundingSphere();
			rig.planLine.geometry = geometry;
			//虚线材质需要逐点累计距离；空几何体没有 position 属性，必须先判断
			if (geometry.attributes.position) {
				rig.planLine.computeLineDistances();
			}
		}
		rig.planLine.visible = planned.length > 1;

		if (isFirst) {
			this.tcpSphere = rig.tcpSphere;
			this.gripper = rig.gripper;
		}
		return rig;
	}

	/// 隐藏一台（已删除的）机械臂的全部模型
	hideRig(rig) {
		rig.pathLine.visible = false;
		rig.planLine.visible = false;
		rig.tcpSphere.visible = false;
		rig.linkPool.forEach(mesh => { mesh.visible = false; });
		rig.jointPool.forEach(mesh => { mesh.visible = false; });
		rig.axisPool.forEach(frame => { frame.group.visible = false; });
		if (rig.gripper) {
			rig.gripper.group.visible = false;
		}
	}

	/// 把夹爪挂到工具末端
	updateGripper_(rig, arm, tcpMatrix) {
		const group = rig.gripper.group;
		if (!arm || !tcpMatrix) {
			group.visible = false;
			return;
		}
		group.visible = true;
		applyMatrix(_mat, tcpMatrix);
		group.position.setFromMatrixPosition(_mat);
		group.quaternion.setFromRotationMatrix(_mat);
	}

	/// 绘制一台机械臂的连杆与关节（与旧的 updateRobot 相同，只是池子按臂分开）
	updateRobotRig(rig, model, tcpMatrix, isPrimary) {
		if (!model) {
			return;
		}
		const joints = model.joints || [];
		const points = joints.map(j => new THREE.Vector3(j.position[0], j.position[1], j.position[2]));
		const linkCount = Math.max(0, points.length - 1);

		//---------- 连杆 ----------
		ensurePool(rig.linkPool, linkCount, () => {
			const mesh = new THREE.Mesh(
				new THREE.CylinderGeometry(1, 1, 1, 16),
				new THREE.MeshStandardMaterial({ color: 0x9fb6d6, roughness: 0.45, metalness: 0.25 }));
			this.armGroup.add(mesh);
			return mesh;
		});
		const colors = model.linkColors || [];
		for (let i = 0; i < linkCount; i++) {
			const mesh = rig.linkPool[i];
			const a = points[i];
			const b = points[i + 1];
			_dir.subVectors(b, a);
			const length = _dir.length();
			if (length < 1e-4) {
				mesh.visible = false;
				continue;
			}
			mesh.visible = true;
			mesh.position.copy(a).addScaledVector(_dir, 0.5);
			_quat.setFromUnitVectors(new THREE.Vector3(0, 1, 0), _dir.clone().normalize());
			mesh.quaternion.copy(_quat);
			const isTool = (i === linkCount - 1);
			const radius = isTool ? 4.5 : 12;
			mesh.scale.set(radius, length, radius);
			mesh.material.color.set(isTool ? TOOL_COLOR : 0x9fb6d6);
			if (!isTool && colors[i]) {
				_color.setRGB(colors[i][0], colors[i][1], colors[i][2]);
				mesh.material.color.copy(_color);
			}
			mesh.material.emissive.copy(mesh.material.color).multiplyScalar(0.12);
		}

		//---------- 关节球 ----------
		ensurePool(rig.jointPool, points.length, () => {
			const mesh = new THREE.Mesh(
				new THREE.SphereGeometry(1, 18, 14),
				new THREE.MeshStandardMaterial({ color: 0xffd166, roughness: 0.35, metalness: 0.3 }));
			this.armGroup.add(mesh);
			return mesh;
		});
		for (let i = 0; i < points.length; i++) {
			const mesh = rig.jointPool[i];
			const isBase = (i === 0);
			const isEnd = (i === points.length - 1);
			mesh.position.copy(points[i]);
			const radius = isBase ? 16 : (isEnd ? 6 : 15);
			mesh.scale.setScalar(radius);
			if (isEnd) {
				mesh.material.color.set(TCP_COLOR);
			}
			else if (isBase) {
				mesh.material.color.set(isPrimary ? 0x7f8ea3 : 0xa3807f);
			}
			else {
				mesh.material.color.set(0xffd166);
			}
		}

		//---------- 关节坐标系 ----------
		const frames = joints.filter(j => j.showAxes);
		ensurePool(rig.axisPool, this.showAxes && this.showToolFrame ? frames.length : 0, () => {
			const frame = this.makeFrame(90, 0xffffff);
			this.armGroup.add(frame.group);
			return frame;
		});
		let axisIndex = 0;
		if (this.showAxes) {
			for (const joint of joints) {
				if (!joint.showAxes) {
					continue;
				}
				const isTool = joint === joints[joints.length - 1];
				if (isTool && !this.showToolFrame) {
					continue;
				}
				if (axisIndex >= rig.axisPool.length) {
					break;
				}
				const frame = rig.axisPool[axisIndex++];
				frame.scale = isTool ? 150 : 100;
				applyMatrix(_mat, joint.matrix);
				this.updateFrame(frame, _mat);
			}
		}
		for (let i = axisIndex; i < rig.axisPool.length; i++) {
			rig.axisPool[i].group.visible = false;
		}

		//---------- 工具末端标记 ----------
		if (tcpMatrix) {
			applyMatrix(_mat, tcpMatrix);
			rig.tcpSphere.position.setFromMatrixPosition(_mat);
		}
	}

	/// 更新有向包围盒（用包围盒自身坐标系的位姿 + 三边长度绘制）
	updateObb(obb) {
		const visible = !this.demoHidden && this.showObb && obb && obb.valid && obb.visible !== false;
		this.obbBox.visible = visible;
		if (!visible) {
			return;
		}
		//BoxGeometry 的边长是 1，因此缩放到三边长度即可；
		//注意必须先平移、再旋转、最后缩放，否则缩放会作用到平移分量上
		const rotation = new THREE.Matrix4().extractRotation(
			applyMatrix(new THREE.Matrix4(), obb.matrix));
		const finalMatrix = new THREE.Matrix4()
			.makeTranslation(obb.center[0], obb.center[1], obb.center[2])
			.multiply(rotation)
			.multiply(new THREE.Matrix4().makeScale(
				obb.extents[0], obb.extents[1], obb.extents[2]));
		this.obbBox.matrixAutoUpdate = false;
		this.obbBox.matrix.copy(finalMatrix);
		this.obbBox.updateMatrixWorld(true);
	}

	/// 用一帧完整状态刷新场景
	update(state) {
		this.state = state;
		if (!state) {
			return;
		}
		//场景运行时隐藏点云 / 包围盒 / 目标坐标系，画面干净一些
		this.demoHidden = !!(state.scenario && state.scenario.running);

		//---------- 多台机械臂 ----------
		const layout = state.scenario && state.scenario.layout;
		if (state.arms && state.arms.length > 0) {
			for (let i = 0; i < state.arms.length; i++) {
				this.updateArmRig(state.arms[i], layout, i === 0, i);
			}
			//删掉臂之后，多出来的绘制装置要整体隐藏（连杆/关节/末梢/夹爪/轨迹线）
			for (let i = state.arms.length; i < this.armRigs.length; i++) {
				this.hideRig(this.armRigs[i]);
			}
		}
		else {
			//兼容：只有单臂字段时按第一台画
			this.updateRobotRig(this.armRig(0, 0x66ccff), state.model, state.tcp && state.tcp.matrix, true);
		}

		this.updateTarget(state.target);
		this.updateTrajectory(state.trajectory);
		this.updateCloud(state.cloud);
		this.updateObb(state.obb);

		//传送带取件场景
		if (state.scenario) {
			this.buildCell(state.scenario.layout, state.arms);
			this.updateBoxes(state.scenario.boxes);
			this.beltSpeed = state.scenario.running ? (state.scenario.beltSpeed || 0) : 0;
		}
	}

	/// 把相机对准整条产线（两台机械臂 + 传送带）
	focus() {
		this.camera.position.set(1150, -2750, 1950);
		this.controls.target.set(620, 0, 280);
		this.controls.update();
	}

	resize() {
		const width = this.container.clientWidth;
		const height = this.container.clientHeight;
		if (width === 0 || height === 0) {
			return;
		}
		this.camera.aspect = width / height;
		this.camera.updateProjectionMatrix();
		this.renderer.setSize(width, height);
	}

	render() {
		//带面纹理滚动，表现皮带在运动（纯视觉，不参与任何运动学计算）
		const dt = Math.min(this.clock.getDelta(), 0.1);
		if (this.beltSpeed && this.beltTexture) {
			const repeat = Math.max(1, this.beltTexture.repeat.y);
			const tileLength = this.beltLength / repeat;
			this.beltTexture.offset.y -= (this.beltSpeed * dt) / tileLength;
		}
		this.controls.update();
		this.renderer.render(this.scene, this.camera);
	}
}
