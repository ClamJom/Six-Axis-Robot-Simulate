#pragma once

//=====================================================================================
// RobotScenario.h
//
// 「传送带两侧各一台机械臂」的取件码垛场景（B/S 架构中的逻辑层，不依赖界面库）。
//
// 场景内容：
//   * 传送带沿 +Y 匀速运动，随机间隔生成随机尺寸/颜色的纸箱，左右分散；
//   * 两侧各一台机械臂（可各用不同的逆解算法），按货物横向位置分工：
//     左臂只管 x < 带面中心线 的货物，右臂只管另一侧；
//   * 每台机械臂有**自己的收集台与统计**，抓取-搬运-码垛的过程互相独立；
//   * 抓取模式可选：货物停稳后抓（定点），或不停机跟踪抓（动态，带运动预测）。
//
// 每台机械臂都会记录**本周期末端轨迹**（TCP 路径点），前端据此画运动轨迹，
// 用来直观对比不同逆解算法的运动品质。
//
// 实现要点：
//   1. 工位在复位时用 Robot::solveIK 预解算，解不出来直接报错，绝不把 NaN 写进关节；
//   2. 运动在**关节空间**平滑插值（MoveJ），动态抓取时改为按速度限幅持续跟踪；
//   3. 抓取位按货物**实际坐标**现解，参考构型取当前关节角，动作幅度最小。
//=====================================================================================

#include "RobotCore.h"

#include <string>
#include <vector>

namespace robotscenario {

	using Vec3 = Eigen::Vector3f;

	/// <summary>一个货物箱</summary>
	struct Box {
		int id = 0;
		Vec3 position = Vec3::Zero();					//中心点（世界坐标）
		Vec3 size = Vec3(140.0f, 140.0f, 140.0f);		//长宽高
		float yaw = 0.0f;								//绕 Z 轴转角
		Vec3 color = Vec3(0.85f, 0.65f, 0.35f);
		enum class State { Belt, Held, Stacked };
		State state = State::Belt;
		float stopY = 0.0f;								//在传送带上的停位
		float stopJitter = 0.0f;						//这一箱的停位抖动（纵向随机）
		/// <summary>
		/// 认领它的机械臂编号（0 = 还没人认领）。
		/// 这就是「锁」：一件货物在传送带上同一时刻只能被一台臂锁定，
		/// 别的臂申请会被拒，从而不会两台臂去抢同一件货。
		/// </summary>
		int claimedBy = 0;
		int slot = 0;									//码垛垛位
		int layer = 0;									//码垛层号
	};

	/// <summary>场景布局（前端渲染也从状态帧里读取这些数值，保证两边一致）</summary>
	struct Layout {
		float beltX = 600.0f;			//传送带中心线 X
		float beltHalfWidth = 170.0f;	//传送带半宽
		float beltTopZ = 200.0f;		//带面高度
		float beltBeginY = -1500.0f;	//生成位置
		float beltEndY = 1500.0f;		//末端（越过即视为已下线）
		float pickY = 0.0f;				//默认工位 Y（每台臂可以各自覆盖，见 Arm::pickY）
		float pickApproachZ = 520.0f;	//取件上方的接近高度
		float pickWindowY = 260.0f;		//动态抓取的有效窗口
		float claimWindowY = 320.0f;	//货物进入工位前多远就可以发出抓取申请
		float pickMissWindowY = 150.0f;	//货物越过工位这么远还没人认领，记一次「从工位前溜走」
		float firstBoxY = -520.0f;		//第一只箱子生成得近一些
		float boxHeight = 100.0f;		//箱子统一高度
		float boxMinFootprint = 80.0f;	//底面边长随机范围
		float boxMaxFootprint = 120.0f;
		float spawnSpreadX = 105.0f;	//生成时的左右随机范围
		float stopJitterY = 20.0f;		//停位的纵向随机范围
		float palletTopZ = 150.0f;		//收集台台面高度
		float layerHeight = 100.0f;		//每层高度
		int layers = 4;					//层数
		int slotCount = 2;				//每台臂的垛位数
		float queueGap = 70.0f;			//排队间隙
	};

	/// <summary>运动阶段（用于状态显示与自检）</summary>
	enum class Phase {
		Idle, ToPick, Descend, Grip, Lift, ToPlace, Lower, Release, Retreat, Fault
	};

	/// <summary>阶段中文名</summary>
	inline const char* phaseName(Phase phase) {
		static const char* names[] = {
			"待命", "前往取件位", "下降", "夹取", "提升",
			"搬运", "下放", "释放", "返回", "故障"
		};
		const int index = static_cast<int>(phase);
		return (index >= 0 && index < 10) ? names[index] : "未知";
	}

	/// <summary>传送带取件场景（可以带多台机械臂）</summary>
	class ConveyorScenario {
	public:
		/// <summary>一台机械臂在场景里的全部状态（身份取自 Robot::ID / Name）</summary>
		struct Arm {
			int id = 0;
			std::string name;
			Robot* robot = nullptr;
			float side = -1.0f;					//-1：基座在带面左侧；+1：右侧（按基座位置自动判定）
			float baseX = 0.0f, baseY = 0.0f;	//基座位置（世界坐标）

			//---------- 每台臂独立的运行参数 ----------
			float speedFactor = 1.0f;			//速度系数
			float trackingSpeed = 1.6f;			//动态跟踪时的最大关节角速度
			bool dynamicPick = false;			//这台臂是否去追移动中的货物
			/// <summary>
			/// 这台臂自己的抓取工位（传送带上的 Y 坐标）。
			/// 货物路过这个位置时，这台臂才会发出抓取申请；
			/// 每台臂的工位可以不同，就形成「上游没抓到、下游接着抓」的流水线。
			/// </summary>
			float pickY = 0.0f;

			//收集台（每台臂自己一套垛位）
			float slotX[2] = { 150.0f, 150.0f };
			float slotY[2] = { -480.0f, -230.0f };
			bool occupied[4][2] = {};

			//运动与状态机
			Eigen::Vector<float, 6> joints = Eigen::Vector<float, 6>::Zero();
			Eigen::Vector<float, 6> moveFrom = Eigen::Vector<float, 6>::Zero();
			Eigen::Vector<float, 6> moveTo = Eigen::Vector<float, 6>::Zero();
			float moveT = 1.0f;
			float moveDuration = 0.0f;
			Phase phase = Phase::Idle;
			int pickBoxId = 0;
			int placeSlot = 0, placeLayer = 0;
			float gripperOpening = 240.0f;
			float gripperTarget = 240.0f;
			float lastPickOffsetX = 0.0f;
			float trackError = 0.0f;			//跟踪过程中的实时水平偏差
			float graspError = 0.0f;			//合爪那一瞬间的水平偏差（衡量抓得准不准）
			std::string message = "未启动";
			bool warning = false;
			float waitTimer = 0.0f;				//在接近位等货物就位的时长（防堵死）
			float retryTimer = 0.0f;			//申请被拒后的退避计时（避免每帧都去撞锁）
			int skipBoxId = 0;					//解算失败过的那件货：不再反复申请它

			//统计
			int picked = 0;
			int stacked = 0;
			int missed = 0;			//从它工位前溜走（还没人认领）的货物
			int cleared = 0;
			int claimed = 0;		//抓取申请成功的次数（拿到锁）
			int contended = 0;		//抓取申请被拒的次数（锁已被别的臂持有）

			//预解算好的工位
			bool waypointsReady = false;
			Eigen::Vector<float, 6> wpReady = Eigen::Vector<float, 6>::Zero();
			Eigen::Vector<float, 6> wpPickUp = Eigen::Vector<float, 6>::Zero();
			Eigen::Vector<float, 6> wpPickDown = Eigen::Vector<float, 6>::Zero();
			struct SlotWaypoints {
				bool ok = false;
				Eigen::Vector<float, 6> up = Eigen::Vector<float, 6>::Zero();
				Eigen::Vector<float, 6> down[4];
			};
			SlotWaypoints slots[2];

			//本周期末端轨迹（前端画运动轨迹用）
			std::vector<Vec3> path;
			int pathRevision = 1;

			//---------- 轨迹规划 ----------
			//旧结算模式的臂走 Robot::getTrack 规划：把规划出的关节序列存下来按时间逐点执行，
			//这样「规划出来的轨迹」与「实际走出来的轨迹」是同一条，调试输出与回放都不会骗人。
			bool usingTrackPlan = false;						//当前这一段是否由 getTrack 规划
			std::vector<Eigen::Vector<float, 6>> plannedPoses;	//getTrack 给出的关节序列
			std::vector<Vec3> planned;							//getTrack 给出的 TCP 路径
			int plannedSegments = 0;							//本周期被 getTrack 规划的段数
			int planTotal = 0;									//累计被 getTrack 规划的段数（不随周期清零）
		};

		ConveyorScenario();

		//---------- 组装 ----------
		/// <summary>
		/// 注册一台机械臂。垛位、负责侧、显示名都按机械臂自身的基座位姿/名称推导，
		/// 因此可以随时再加一台（摆在哪就负责哪一片），不需要改场景代码。
		/// </summary>
		Arm& addArm(Robot& robot);

		/// <summary>移除一台机械臂（它抓着的/码好的货物一并清掉）</summary>
		bool removeArm(int armId);

		/// <summary>按编号找一台臂，找不到返回 nullptr</summary>
		Arm* armById(int armId);
		const Arm* armById(int armId) const;

		/// <summary>移动机械臂基座（世界坐标）：重算垛位与工位，清空它自己的收集台</summary>
		bool setArmBase(int armId, float x, float y);

		/// <summary>设置某台臂的抓取工位（传送带上的 Y 坐标）</summary>
		bool setArmPickY(int armId, float y);

		//---------- 货物锁（认领机制）----------
		/// <summary>
		/// 某台臂对某件货物发出抓取申请：
		///   没人认领 → 认领成功（置 claimedBy）返回 true；
		///   已经是自己认领的 → true；
		///   已被别的臂认领 → 记一次 contended 并返回 false（调用方应当放弃这一件）。
		/// </summary>
		bool requestBox(int armId, int boxId);

		/// <summary>释放某台臂对某件货物的认领（放弃/失败时调用，货物继续流向下游）</summary>
		void releaseBox(int armId, int boxId);

		/// <summary>释放某台臂持有的全部认领（复位、删除臂、挪动基座时用）</summary>
		void releaseClaims(int armId);

		//---------- 控制 ----------
		void start();
		void pause();
		void setBeltSpeed(float speed) { beltSpeed_ = clampf(speed, 40.0f, 600.0f); }
		void setSpawnRange(float minSeconds, float maxSeconds);
		void setAutoClear(bool enabled) { autoClear_ = enabled; }
		void setDynamicPick(bool enabled);
		void setTrackingSpeed(float speed);

		/// <summary>设置某台臂的参数；armId 传 -1 表示所有臂</summary>
		void setArmSpeed(int armId, float factor);
		void setArmTrackingSpeed(int armId, float speed);
		void setArmDynamicPick(int armId, bool enabled);

		/// <summary>
		/// 轨迹采样点数：旧特解模式的臂按 Robot::getTrack 规划的关节序列逐点执行，
		/// 点数越多动作越连续（点数少时每一段只有寥寥几个离散姿态，看着会"咯噔"）。
		/// armId 传 -1 表示所有臂。
		/// </summary>
		void setTrackSize(int size, int armId = -1);
		int trackSize() const { return trackSize_; }

		/// <summary>清空货物与统计，重新预解算工位，各臂回到待命位</summary>
		void reset();

		/// <summary>推进一帧仿真（dt 秒）</summary>
		void tick(float dt);

		//---------- 查询 ----------
		bool running() const { return running_; }
		Layout& layout() { return layout_; }
		const Layout& layout() const { return layout_; }
		const std::vector<Box>& boxes() const { return boxes_; }
		const std::vector<Arm>& arms() const { return arms_; }
		std::vector<Arm>& arms() { return arms_; }
		int armCount() const { return static_cast<int>(arms_.size()); }
		float beltSpeed() const { return beltSpeed_; }
		float spawnMin() const { return spawnMin_; }
		float spawnMax() const { return spawnMax_; }
		bool autoClear() const { return autoClear_; }
		bool dynamicPick() const { return dynamicPick_; }
		int onBeltCount() const;
		int capacity() const { return layout_.layers * layout_.slotCount; }
		const std::string& message() const { return message_; }
		bool warning() const { return warning_; }

	private:
		//---------- 世界 ----------
		Layout layout_;
		std::vector<Box> boxes_;
		std::vector<Arm> arms_;
		int nextId_ = 1;
		bool firstBox_ = true;

		bool running_ = false;
		bool autoClear_ = true;
		bool dynamicPick_ = false;						//默认抓取模式（新注册的臂继承它）
		int trackSize_ = 24;							//默认轨迹采样点数（4~200）
		float graspTolerance_ = 14.0f;
		float beltSpeed_ = 160.0f;
		float spawnMin_ = 3.0f;
		float spawnMax_ = 5.5f;
		float spawnTimer_ = 0.8f;
		float time_ = 0.0f;
		std::string message_ = "未启动";
		bool warning_ = false;

		//---------- 内部实现 ----------
		static float clampf(float value, float low, float high) {
			return (value < low) ? low : ((value > high) ? high : value);
		}

		/// <summary>构造「工具竖直向下」的位姿（X 轴对齐世界 X 轴）</summary>
		static Eigen::Matrix4f topDownPose(const Vec3& position);

		/// <summary>预解算与货物无关的工位（待命位、各垛位）</summary>
		bool prepareWaypoints(Arm& arm, std::string& error);

		/// <summary>解一个「工具竖直向下、落在 (x,y,z)」的位姿，带小范围试探</summary>
		bool solveTopDownAt(Arm& arm, float x, float y, float z,
			const Eigen::Vector<float, 6>& reference, Eigen::Vector<float, 6>& out) const;

		/// <summary>按货物实际坐标现解取件位与接近位</summary>
		bool preparePickWaypoints(Arm& arm, const Box& box, std::string& error);

		/// <summary>按最大关节角速度朝目标构型「赶」：动态跟踪专用</summary>
		void steerTo(Arm& arm, const Eigen::Vector<float, 6>& target, float maxJointSpeed, float dt);

		void beginMove(Arm& arm, const Eigen::Vector<float, 6>& target, float speed);
		bool advanceMove(Arm& arm, float dt);
		void applyJoints(Arm& arm);

		/// <summary>选择下一个空闲垛位</summary>
		bool findFreeSlot(Arm& arm, int& slot, int& layer) const;

		/// <summary>找一件「正经过我的工位、还没被别人锁定」的货物（最靠前的那件）</summary>
		Box* findClaimableBox(const Arm& arm);

		/// <summary>是否还有另一台空闲且横向更近的臂更适合抓这件货</summary>
		bool betterSuited(const Arm& arm, const Box& box) const;

		/// <summary>窗口内找货：mode 0 = 未被认领；mode 1 = 已被别的臂认领</summary>
		Box* findBoxInWindow(const Arm& arm, int mode);

		Box* findBox(int id);
		const Box* findBox(int id) const;

		/// <summary>记录本周期末端轨迹</summary>
		void recordPath(Arm& arm);

		void spawnBox();
		void updateBelt(float dt);
		void updateGripper(Arm& arm, float dt);
		void updateArm(Arm& arm, float dt);
		void updateHeldBoxes();
		const Box* leadBoxForTracking(const Arm& arm) const;
		float horizontalError(const Arm& arm, const Box& box) const;

		void fail(Arm& arm, const std::string& reason);
	};

}
