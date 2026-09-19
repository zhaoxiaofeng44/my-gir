# hair_physics.gd — HS2 头发弹性物理（自实现 verlet 发链）
#
# 为什么不用引擎的 SpringBoneSimulator3D：
#   本工程的骨来自 FBX→glb 转换，骨轴不是标准朝向；实测内置模拟器
#   （链已识别 4/4/5/5/6/6/6 关节、active=true）完全不驱动骨架（像素差 0）。
#   因此这里自己实现：对每条发链做「尾点 verlet 积分 + 回弹 + 阻尼 + 重力 + 外力」，
#   再把结果写回骨骼的 pose 旋转。可在 headless 下验证（_physics_process 会跑）。
#
# 发链（都挂在 cf_J_Head_s 下）：
#   前发 cf_J_hairF_top → {FL_01, FR_01, FSL_02, FSR_02}
#   后发 cf_J_hairB_top → {BC_03, BL_03, BR_03}
class_name HS2HairPhysics
extends Node

const CHAINS := [
	{"root": "cf_J_hairF_top", "end": "cf_J_hairFL_01"},
	{"root": "cf_J_hairF_top", "end": "cf_J_hairFR_01"},
	{"root": "cf_J_hairF_top", "end": "cf_J_hairFSL_02"},
	{"root": "cf_J_hairF_top", "end": "cf_J_hairFSR_02"},
	{"root": "cf_J_hairB_top", "end": "cf_J_hairBC_03"},
	{"root": "cf_J_hairB_top", "end": "cf_J_hairBL_03"},
	{"root": "cf_J_hairB_top", "end": "cf_J_hairBR_03"},
]

const CENTER_BONE := "cf_J_Head_s"

@export var enabled := true
@export_range(0.0, 1.0) var stiffness := 0.35    # 回弹：越大越贴回静止姿态
@export_range(0.0, 1.0) var drag := 0.60        # 阻尼：越大衰减越快（0.6 ≈ ζ0.35，1~2 次回摆）
@export_range(0.0, 1.0) var gravity := 0.05     # 重力（0..1 → 0..14 m/s²，0.05 约 0.7 m/s² 微垂）
@export_range(-15.0, 15.0) var wind := 0.0      # 风力（横向，m/s²）
@export var inertia := 1.0                      # 惯性倍率
@export var substeps := 2                       # 每帧子步数（稳定性）
@export var head_collision_radius := 0.095      # 头/身碰撞半径（防止穿头）

var steps := 0
var skel: Skeleton3D
var _chains: Array = []      # [[joint...]]
var _ok := false

func setup(skeleton: Skeleton3D) -> void:
	skel = skeleton
	_chains.clear()
	_ok = false
	if skel == null:
		return
	var built := 0
	for c in CHAINS:
		var path := _bone_path(c["root"], c["end"])
		if path.size() < 2:
			continue
		var joints: Array = []
		for i in path.size():
			var b: int = path[i]
			var rest := skel.get_bone_rest(b)
			var next_b: int = path[i + 1] if i + 1 < path.size() else -1
			var length := 0.0
			var axis := Vector3.ZERO
			if next_b >= 0:
				# 注意：get_bone_rest() 返回的是**局部** rest，
				# 子骨局部 rest 的 origin 即「相对本骨原点的偏移」，不要再减本骨 origin。
				var off: Vector3 = skel.get_bone_rest(next_b).origin
				length = off.length()
				axis = off.normalized()
			else:
				# 末端骨（叶子）：沿父级方向延伸，避免凭空造轴
				var prev_b: int = path[i - 1] if i > 0 else -1
				if prev_b >= 0:
					var pa: Vector3 = _axis_local(prev_b)
					var prest: Basis = skel.get_bone_rest(prev_b).basis
					axis = (rest.basis.inverse() * (prest * pa)).normalized()
					var pch := _first_child_bone(prev_b)
					length = skel.get_bone_rest(pch).origin.length() if pch >= 0 else 0.03
				else:
					length = 0.03
					axis = Vector3(0, 1, 0)
			var head_rest: Vector3 = skel.get_bone_global_rest(b).origin
			var gb: Basis = skel.get_bone_global_rest(b).basis
			joints.append({
				"bone": b,
				"parent": skel.get_bone_parent(b),
				"len": length,
				"axis": axis,
				"head_rest": head_rest,
				"tail_rest": head_rest + gb * axis * length,
				"tail": head_rest + gb * axis * length,
				"prev": head_rest + gb * axis * length,
				"vel": Vector3.ZERO,
			})
		_chains.append(joints)
		built += 1
	_ok = built > 0
	_reset()
	print("[HairPhysics] 自实现发链 ", built, " 条（关节数 ", _joint_total(), "）")

func _joint_total() -> int:
	var n := 0
	for j in _chains:
		n += j.size()
	return n

func _bone_path(root: String, endb: String) -> Array:
	var ri := skel.find_bone(root)
	var ei := skel.find_bone(endb)
	if ri < 0 or ei < 0:
		return []
	var path: Array = []
	var cur := ei
	while cur >= 0:
		path.push_front(cur)
		if cur == ri:
			break
		cur = skel.get_bone_parent(cur)
	if path.is_empty() or path[0] != ri:
		return []
	return path

func _axis_local(b: int) -> Vector3:
	# 该骨指向其第一个子骨的单位方向（骨局部空间；子骨局部 rest 的 origin 就是偏移）
	var ch := _first_child_bone(b)
	if ch < 0:
		return Vector3(0, 1, 0)
	var off: Vector3 = skel.get_bone_rest(ch).origin
	if off.length() < 1e-6:
		return Vector3(0, 1, 0)
	return off.normalized()

func _first_child_bone(b: int) -> int:
	for i in skel.get_bone_count():
		if skel.get_bone_parent(i) == b:
			return i
	return -1

func _reset() -> void:
	for joints in _chains:
		for j in joints:
			var p := skel.get_bone_global_rest(j["bone"]).origin
			var d := skel.get_bone_global_rest(j["bone"]).basis * Vector3(j["axis"])
			j["tail"] = p + d * float(j["len"])
			j["prev"] = j["tail"]
			j["vel"] = Vector3.ZERO

func set_enabled(on: bool) -> void:
	enabled = on
	if skel == null:
		return
	if on:
		reset_physics()
	else:
		_restore_rest()

func reset_physics() -> void:
	_restore_rest()
	_reset()

func _restore_rest() -> void:
	for joints in _chains:
		for j in joints:
			skel.reset_bone_pose(j["bone"])

func _physics_process(delta: float) -> void:
	if not _ok or not enabled or skel == null:
		return
	steps += 1
	var sub := maxi(substeps, 1)
	var dt := minf(delta, 0.05) / float(sub)
	for i in sub:
		_step(dt)

# 弹簧-阻尼（速度积分）：accel = 回弹 + 重力 + 风；vel 阻尼衰减；tail += vel*dt
func _step(dt: float) -> void:
	var head_idx := skel.find_bone(CENTER_BONE)
	var head_center: Vector3 = skel.get_bone_global_pose(head_idx).origin if head_idx >= 0 else Vector3.ZERO
	var k := stiffness_k()
	var damp := drag_k()
	var g := gravity_k()
	for joints in _chains:
		# 1) 尾点积分
		for j in joints:
			var b: int = j["bone"]
			var cur_head: Vector3 = skel.get_bone_global_pose(b).origin
			var parent: int = j["parent"]
			var gb: Basis = skel.get_bone_global_rest(b).basis
			var rest_dir: Vector3 = gb * Vector3(j["axis"])
			var rigid_dir := rest_dir
			if parent >= 0:
				var pdelta: Basis = skel.get_bone_global_pose(parent).basis * skel.get_bone_global_rest(parent).basis.inverse()
				rigid_dir = pdelta * rest_dir
			var rigid_tail: Vector3 = cur_head + rigid_dir * float(j["len"])
			var accel: Vector3 = (rigid_tail - Vector3(j["tail"])) * k
			accel += Vector3(0.0, -1.0, 0.0) * g
			accel += Vector3(wind, 0.0, 0.0)
			var vel: Vector3 = Vector3(j["vel"]) * maxf(1.0 - damp * dt, 0.0) + accel * dt
			var nxt: Vector3 = Vector3(j["tail"]) + vel * dt
			# 头/身碰撞：把尾点推离头骨球
			var to_tail: Vector3 = nxt - head_center
			var d: float = to_tail.length()
			if d > 1e-6 and d < head_collision_radius:
				nxt = head_center + to_tail / d * head_collision_radius
			# 长度约束（骨长不变）
			var dirv: Vector3 = nxt - cur_head
			if dirv.length() > 1e-6:
				nxt = cur_head + dirv.normalized() * float(j["len"])
			j["vel"] = vel
			j["tail"] = nxt
		# 2) 由尾点反求 pose 旋转（自顶向下，父级先写）
		for j in joints:
			var b: int = j["bone"]
			var cur_head: Vector3 = skel.get_bone_global_pose(b).origin
			var want: Vector3 = Vector3(j["tail"]) - cur_head
			if want.length() < 1e-6:
				continue
			var parent: int = j["parent"]
			var pgb: Basis = skel.get_bone_global_pose(parent).basis if parent >= 0 else Basis()
			var want_parent: Vector3 = (pgb.inverse() * want.normalized()).normalized()
			var axis: Vector3 = Vector3(j["axis"])
			# 关键：以「静止方向（父空间）」为基准做最小旋转，再乘静止姿态，
			# 这样既对准方向、又保留静止姿态绕骨轴的扭转（否则 hair card 会被拧转）
			var rest_basis: Basis = skel.get_bone_rest(b).basis
			var rest_dir_parent: Vector3 = (rest_basis * axis).normalized()
			var q_delta := Quaternion(rest_dir_parent, want_parent)
			var target: Quaternion = q_delta * rest_basis.get_rotation_quaternion()
			# 全量写入：尾点积分本身已平滑，再叠加插值会与「头位置随 pose 变化」形成反馈振荡
			skel.set_bone_pose_rotation(b, target)

func stiffness_k() -> float:
	return lerpf(25.0, 600.0, stiffness)

func drag_k() -> float:
	return lerpf(1.0, 16.0, drag)

func gravity_k() -> float:
	return gravity * 14.0