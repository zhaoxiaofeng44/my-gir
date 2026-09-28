extends Node3D
## XPBD 布料交互测试场景(管理器): 多布料 + 可拖拽碰撞体 + 自碰撞 + 布布互碰 + 物体碰撞
## 左键: 点中碰撞体->拖动它(按住时 Q/E 旋转, 可直接穿过布料砸) | 否则: 抓取布料(甩动可抛出)
## 右键拖动: 旋转相机 | 滚轮: 缩放 | R: 重置 | 面板: 实时参数 + 碰撞体形状 + 布料数量
## 防穿透: 碰撞体局部空间解析TOI(球/胶囊/方箱) + 切向滑移摩擦 + 最终最小穿透推出兜底(见 XPBDCloth/XPBDCollider)

@export var substeps: int = 4
## 布料求解后端: true = C++ GDExtension(快 ~17x), false = 纯 GDScript(参考实现)
@export var use_native_backend: bool = true
@export var gravity: Vector3 = Vector3(0, -9.8, 0)
@export var wind_strength: float = 0.0
@export var enable_object_collision: bool = true
@export var enable_self_collision: bool = true
@export var enable_cloth_collision: bool = true
@export var grab_stiffness: float = 0.6

# ---------- 布料(多实例) ----------
var cloths: Array = []          # [XPBDCloth]
var cloth_count: int = 2
# 面板权威参数(重建布料后依然生效)
var air_drag := 0.005
var friction := 0.85
var cloth_thickness := 0.035
var max_stretch := 1.06
var softness := 0.5

# ---------- 碰撞体(可拖拽) ----------
var collider := XPBDCollider.new()
var blob_root: Node3D
var blob_center := Vector3(0, 0.2, 0)
var blob_yaw: float = 0.0
var blob_moving: bool = false
var blob_t: float = 0.0
var collider_mode: int = 0      # 0岩石 1胶囊 2方箱
var ground_y: float = -1.6

# ---------- 相机 / 交互 ----------
var cam: Camera3D
var cam_yaw: float = -0.45
var cam_pitch: float = -0.35
var cam_dist: float = 3.4
var cam_target := Vector3(0, 0.2, 0)
var drag_collider: bool = false
var drag_offset := Vector3.ZERO
var drag_depth: float = 0.0
var drag_pos := Vector3.ZERO      # 拖拽目标位置(限速逼近)
var drag_speed := 2.5             # 碰撞体拖拽限速 m/s: 保证每帧位移 < 布厚量级 => TOI+推出稳定
var grabbed_cloth: Node3D = null

var _frame_count: int = 0
var pen_count: int = 0          # 物体穿透审计(应恒为0)
var pen_inter: int = 0          # 布布穿透审计
var _fast_sticky: bool = false  # 接触每子步求解的滞回状态(受压触发后保持到充分回稳)

func _ready() -> void:
	Engine.max_physics_steps_per_frame = 4
	get_window().always_on_top = true
	get_window().position = Vector2i(60, 60)
	get_window().move_to_foreground()
	get_window().grab_focus()
	cam = Camera3D.new()
	add_child(cam)
	_update_cam()
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-45, 30, 0)
	add_child(light)
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.15, 0.17, 0.2)
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.7, 0.7, 0.75)
	e.ambient_light_energy = 0.8
	add_child(env)
	env.environment = e

	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(8, 8)
	ground.mesh = plane
	ground.position.y = ground_y
	var gm := StandardMaterial3D.new()
	gm.albedo_color = Color(0.25, 0.28, 0.32)
	ground.material_override = gm
	add_child(ground)

	_build_blob()
	_rebuild_cloths()
	_build_ui()

# ---------- 碰撞体构建(视觉与 XPBDCollider 局部空间严格一致) ----------
func _build_blob() -> void:
	if blob_root:
		blob_root.free()
	blob_root = Node3D.new()
	add_child(blob_root)
	collider.root = blob_root
	collider.clear()
	var bm := StandardMaterial3D.new()
	bm.albedo_color = Color(0.45, 0.42, 0.4)
	bm.roughness = 0.95
	if collider_mode == 0:
		var defs := [
			[Vector3(0, 0, 0), 0.42], [Vector3(0.26, 0.12, 0.08), 0.30],
			[Vector3(-0.24, 0.16, -0.06), 0.28], [Vector3(0.04, -0.22, 0.14), 0.30],
			[Vector3(0.08, 0.06, -0.28), 0.27], [Vector3(-0.1, -0.08, 0.26), 0.26]]
		for d in defs:
			collider.add_sphere(d[0], d[1])
			var mi := MeshInstance3D.new()
			var sm := SphereMesh.new()
			sm.radius = d[1]
			sm.height = d[1] * 2.0
			mi.mesh = sm
			mi.position = d[0]
			mi.material_override = bm
			blob_root.add_child(mi)
	elif collider_mode == 1:
		var a := Vector3(0, 0.5, 0)
		var b := Vector3(0.35, -0.5, 0.2)
		collider.add_capsule(a, b, 0.38)
		var mi := MeshInstance3D.new()
		var cm := CapsuleMesh.new()
		cm.radius = 0.38
		cm.height = a.distance_to(b) + 0.76
		mi.mesh = cm
		mi.position = (a + b) * 0.5
		mi.rotation = Quaternion(Vector3.UP, (b - a).normalized()).get_euler()
		mi.material_override = bm
		blob_root.add_child(mi)
	else:
		var half := Vector3(0.45, 0.3, 0.45)
		collider.add_box(Vector3.ZERO, half)
		var mi := MeshInstance3D.new()
		var bx := BoxMesh.new()
		bx.size = half * 2.0
		mi.mesh = bx
		mi.material_override = bm
		blob_root.add_child(mi)
	blob_root.position = blob_center
	blob_root.rotation.y = blob_yaw

# ---------- 布料实例: 落体 + 挂帘x2 ----------
func _rebuild_cloths() -> void:
	for cl in cloths:
		cl.free()
	cloths.clear()
	add_cloth(_make_sheet())
	if cloth_count >= 2:
		add_cloth(_make_curtain(Vector3(-1.35, 2.05, -0.6), 0.0, Color(0.3, 0.5, 0.9)))
	if cloth_count >= 3:
		add_cloth(_make_curtain(Vector3(0.1, 2.05, 1.5), PI * 0.5, Color(0.35, 0.8, 0.45)))
	if cloth_count >= 4:
		add_cloth(_make_drape(Vector3(0.15, 2.6, -0.1), Vector2(1.9, 1.9), Color(0.9, 0.55, 0.2), "Drape4"))
	if cloth_count >= 5:
		add_cloth(_make_drape(Vector3(-0.12, 3.4, 0.15), Vector2(1.8, 1.8), Color(0.62, 0.42, 0.85), "Drape5"))
	_sync_params()
	_refresh_native_impl()

func _make_sheet() -> Node3D:
	return _make_drape(Vector3(0, 1.8, 0), Vector2(2.0, 2.0), Color(0.85, 0.3, 0.35), "Sheet", 24)

const XPBDClothNativeAdapterScript := preload("res://scripts/xpbd_cloth_native_adapter.gd")

# 布料工厂: 按 use_native_backend 选择实现(两者接口一致)
func _new_cloth() -> Node3D:
	if use_native_backend and ClassDB.class_exists("XPBDClothNative"):
		return XPBDClothNativeAdapterScript.new()
	return XPBDCloth.new()

func _make_drape(origin: Vector3, size: Vector2, col: Color, cname: String, res: int = 20) -> Node3D:
	var cl := _new_cloth()
	cl.name = cname
	cl.cols = res
	cl.rows = res
	cl.cloth_size = size
	cl.spawn_origin = origin
	cl.albedo = col
	add_child(cl)
	cl.build()
	return cl

func _make_curtain(origin: Vector3, yaw: float, col: Color) -> Node3D:
	var cl := _new_cloth()
	cl.name = "Curtain"
	cl.cols = 16
	cl.rows = 16
	cl.cloth_size = Vector2(2.0, 1.7)
	cl.spawn_origin = origin
	cl.vertical = true
	cl.yaw = yaw
	cl.albedo = col
	add_child(cl)
	cl.build()
	return cl

func add_cloth(cl: Node3D) -> void:
	cl.colliders = [collider]
	cloths.append(cl)

func _sync_params() -> void:
	for cl in cloths:
		cl.gravity = gravity
		cl.enable_object_collision = enable_object_collision
		cl.enable_self_collision = enable_self_collision
		cl.ground_y = ground_y
		cl.grab_stiffness = grab_stiffness
		cl.air_drag = air_drag
		cl.friction = friction
		cl.cloth_thickness = cloth_thickness
		cl.max_stretch = max_stretch
		cl.compliance_bend = lerpf(0.0000001, 0.0003, softness)
		cl.compliance_shear = lerpf(0.0000001, 0.00002, softness)

# ---------- 模拟编排 ----------
func _run_sim(delta: float) -> void:
	_frame_count += 1
	if drag_collider:
		# 限速逼近鼠标目标: 每步位移小于布厚量级, TOI+推出兜底才能稳定处理"碰撞体穿布"
		# 限位: 不把碰撞体压到与地面形成 < 2.5倍布厚的缝 — 布料被夹进亚布厚缝隙会被
		# 揉成永久手风琴褶皱(无展开恢复力), 这是"压扁成一团"的根源
		var half_y := 0.45
		if collider_mode == 1:
			half_y = 0.88
		elif collider_mode == 2:
			half_y = 0.30
		drag_pos.y = maxf(drag_pos.y, ground_y + half_y + cloth_thickness * 2.5)
		var to := drag_pos - blob_center
		var step := drag_speed * delta
		blob_center = blob_center + to.normalized() * step if to.length() > step else drag_pos
		blob_root.position = blob_center
	elif blob_moving:
		blob_t += delta
		blob_center = Vector3(0.35 * sin(blob_t * 0.8), 0.2 + 0.1 * sin(blob_t * 1.3), 0.25 * cos(blob_t * 0.6))
		blob_yaw += delta * 0.6
		blob_root.position = blob_center
		blob_root.rotation.y = blob_yaw
	var h := delta / maxf(substeps, 1)
	var n_sub: int = maxi(substeps, 1)
	# 自适应接触频率: 顶点高速(平方>4 即 >2m/s)、碰撞体被驱动、或堆叠受压(欠量>0.015)时,
	# 自碰+布间升级为每子步求解 — 接触网络与约束同频是密集堆叠不深穿的最低配置
	# 滞回: 触发后保持每子步直到欠量 < 0.005, 避免在 0.015 门限附近抖动(压紧折皱静置时
	# 每帧只有最后子步解接触, 重力反复再压入, 会稳态滞留 ~1.5cm 夹层)
	# 紧迫度用"基准间距"度量(不含几何补偿): 补偿是静态安全余量, 不代表每子步都必须求解。
	# 用补偿后的欠量做门限会让 fast 恒为真 => 每子步全量求解(5层实测 407ms/帧)。
	var base_sep: float = cloth_thickness * 2.0
	# 风载时布面持续被推挤, 布间只在末子步解会残留亚毫米级穿插(实测 E1 长跑 0.7mm);
	# 有风即升级为每子步求解(风载场景本就少布料, 成本可接受)
	var fast := drag_collider or blob_moving or _fast_sticky or wind_strength > 0.01
	for cl in cloths:
		if cl.max_vel2 > 4.0 or cl.contact_deficit > base_sep * 0.35:
			fast = true
		cl.max_vel2 = 0.0
	# 滞回解除: 全布料欠量充分小且碰撞体空闲时才回落到末子步求解
	var worst_deficit := 0.0
	for cl in cloths:
		worst_deficit = maxf(worst_deficit, cl.contact_deficit)
	_fast_sticky = fast and (worst_deficit >= base_sep * 0.15 or drag_collider or blob_moving)
	var wind_vec := Vector3(0, 0, wind_strength) * (0.6 + 0.4 * sin(Time.get_ticks_msec() / 700.0))
	for sub_i in n_sub:
		for cl in cloths:
			cl.sub_begin()
			cl.sub_integrate(h, wind_vec)
			cl.sub_solve(h)
		# 自碰撞/布布碰撞: 低速每帧最后子步一次; 高速/驱动时每子步
		var last := sub_i == n_sub - 1
		var contacts := last or fast
		if contacts:
			for cl in cloths:
				cl.begin_contacts()
			# 自碰: 快时每子步(折叠接触与约束同频), 慢时仅末子步
			for cl in cloths:
				cl.self_collision()
			# 布间在"快"(驱动/高速/受压)时也必须每子步: 6m/s 砸穿会剧烈压紧布层,
			# 只靠末子步分离会把整叠布挤进碰撞体(实测物体穿透 0.0245m 回归)
			if fast:
				_cross_contact_any(true)
		for cl in cloths:
			if not contacts:
				cl.begin_contacts()
			cl.object_collision()
		# 末子步: 顺序 "物体推出 -> 布间 -> 自碰 -> 物体复检"。
		# 顺序反了(接触->物体推出)会把刚分开的顶点推回对层布里(实测残留 33mm)。
		# 慢速时布间在此处解一次即可(层间是慢过程), 避免每子步 4 次的开销。
		if last:
			for cl in cloths:
				cl.final_push_out()
			if not fast:
				_cross_contact_any(true)
			for cl in cloths:
				cl.self_collision()
			for cl in cloths:
				cl.final_push_out()
		for cl in cloths:
			cl.sub_recover(h)

# 布间调度: C++ 后端时走原生(内部维护全局注册表), 否则回落 GDScript 实现
func _cross_contact_any(push: bool) -> void:
	if use_native_backend and _native_impl != null:
		_native_impl.cross_contact_all(push)
		return
	_cross_contact(push)

# 返回首个原生布料实例(用于调用其跨布静态接口), 无则 null
var _native_impl = null

func _refresh_native_impl() -> void:
	_native_impl = null
	if not use_native_backend:
		return
	for cl in cloths:
		if cl.get("native") != null:
			_native_impl = cl.native
			return

# ---------- 布间面级碰撞(多层叠放防穿模) ----------
# 顶点式碰撞的盲区: 一层的顶点全在另一层外侧, 但它的表面(三角形)已从另一层中间穿过。
# 5 层叠放实测: 顶点口径 0.00mm 合格, 面级口径却有 35mm 穿插(7466 采样点命中)。
# 实现(性能关键): 不做第二次全量哈希。直接用各布顶点自身的"关联三角形"CSR 表 —
# 对 A 的每个顶点, 只测试对层中"距该顶点最近的顶点们所关联的三角形"(1 环邻域), 覆盖
# 该处的表面; 每子步成本 ~ O(界面相邻顶点数), 与顶点级碰撞同量级。
# 复用跨布顶点哈希(与顶点级布间同一次遍历): 已检出近邻对 (i in A, j in B) 时,
# 顺带把 i 的顶点从 B 中 j 的关联三角形表面推开 — 消除"顶点扎进另一层表面"的穿模,
# 额外成本仅为每对 ~2-6 次点-三角测试(拓扑表静态预计算)。
func _cross_face_contact() -> void:
	if not enable_cloth_collision or cloths.size() < 2:
		return
	var min_d: float = cross_min_d()
	var half: float = min_d * 0.5
	var inv := 1.0 / min_d
	# 已由 _cross_contact 内的"外接半径补偿距离"从根上保证层间表面不相交,
	# 此函数保留为可选加固入口(默认不启用, 避免每子步 ~42ms 开销)
	pass

# 把 A 的顶点 i 从 B 中顶点 k 的关联三角形表面推开(反作用分摊给三角形三顶点)
func _face_push_for(A, ai: int, i: int, B, bi: int, k: int, half: float, smin2: float) -> void:
	var p: Vector3 = A.pos[i]
	var wai := 0.0 if (A.pinned[i] == 1 or (A.grabbing and i == A.grab_id)) else 1.0
	for t in range(B.vtx_tri_ofs[k], B.vtx_tri_ofs[k + 1]):
			var tid: int = B.vtx_tri_ids[t]
			var i0: int = B.tri_verts[tid * 3]
			var i1: int = B.tri_verts[tid * 3 + 1]
			var i2: int = B.tri_verts[tid * 3 + 2]
			var p0: Vector3 = B.pos[i0]
			var p1: Vector3 = B.pos[i1]
			var p2: Vector3 = B.pos[i2]
			var cp := _closest_on_tri(p, p0, p1, p2)
			var d := p - cp
			var d2 := d.length_squared()
			if d2 >= smin2:
				continue
			var dist := sqrt(maxf(d2, 0.0))
			var nrm := Vector3.ZERO
			if dist > 1e-9:
				nrm = d / dist
			else:
				nrm = (p1 - p0).cross(p2 - p0)
				nrm = nrm.normalized() if nrm.length_squared() > 1e-18 else Vector3.UP
			var deficit := half - dist
			if deficit <= 0.0:
				continue
			var w0 := 0.0 if (B.pinned[i0] == 1 or (B.grabbing and i0 == B.grab_id)) else 1.0
			var w1 := 0.0 if (B.pinned[i1] == 1 or (B.grabbing and i1 == B.grab_id)) else 1.0
			var w2 := 0.0 if (B.pinned[i2] == 1 or (B.grabbing and i2 == B.grab_id)) else 1.0
			var wb := w0 + w1 + w2
			var wt := wai + wb
			if wt <= 0.0:
				continue
			var mv := deficit / wt
			A.pos[i] += nrm * mv * wai
			p = A.pos[i]
			A._pushed_soft[i] = 1
			face_pushes += 1
			if w0 > 0.0:
				B.pos[i0] -= nrm * mv * w0
				B._pushed_soft[i0] = 1
			if w1 > 0.0:
				B.pos[i1] -= nrm * mv * w1
				B._pushed_soft[i1] = 1
			if w2 > 0.0:
				B.pos[i2] -= nrm * mv * w2
				B._pushed_soft[i2] = 1

# 点到三角形最近点
func _closest_on_tri(p: Vector3, a: Vector3, b: Vector3, c: Vector3) -> Vector3:
	var ab := b - a
	var ac := c - a
	var ap := p - a
	var d1 := ab.dot(ap)
	var d2 := ac.dot(ap)
	if d1 <= 0.0 and d2 <= 0.0:
		return a
	var bp := p - b
	var d3 := ab.dot(bp)
	var d4 := ac.dot(bp)
	if d3 >= 0.0 and d4 <= d3:
		return b
	var vc := d1 * d4 - d3 * d2
	if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
		return a + ab * (d1 / (d1 - d3))
	var cp := p - c
	var d5 := ab.dot(cp)
	var d6 := ac.dot(cp)
	if d6 >= 0.0 and d5 <= d6:
		return c
	var vb := d5 * d2 - d1 * d6
	if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
		return a + ac * (d2 / (d2 - d6))
	var va := d3 * d6 - d5 * d4
	if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
		var t := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return b + (c - b) * t
	var denom := 1.0 / (va + vb + vc)
	return a + ab * (vb * denom) + ac * (vc * denom)

# 面级布间欠量实测(与求解同口径, 不推顶点): 返回最深欠量(m), 0 表示无穿插。
# 复用跨布顶点哈希: 只有哈希判定为近邻(距离 < 2*min_d)的对才做点-三角测试。
func face_deficit() -> float:
	if cloths.size() < 2:
		return 0.0
	var half: float = cloths[0].cloth_thickness * 2.0 * 0.5
	var min_d: float = half * 2.0
	var inv := 1.0 / (min_d * 2.0)
	_cross_build(inv)
	var worst := 0.0
	var smin2 := (half * 1.5) * (half * 1.5)
	for g in _owner_of.size():
		var ci: int = _owner_of[g]
		var A = cloths[ci]
		var i: int = _local_of[g]
		var p: Vector3 = A.pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		for off in XPBDCloth._OFFS:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if not _xhash.has(key):
				continue
			for jj in _xhash[key]:
				var j: int = jj
				var cj: int = _owner_of[j]
				if cj == ci:
					continue
				var B = cloths[cj]
				var k: int = _local_of[j]
				for t in range(B.vtx_tri_ofs[k], B.vtx_tri_ofs[k + 1]):
					var tid: int = B.vtx_tri_ids[t]
					var cp := _closest_on_tri(p, B.pos[B.tri_verts[tid * 3]], B.pos[B.tri_verts[tid * 3 + 1]], B.pos[B.tri_verts[tid * 3 + 2]])
					var d2 := p.distance_squared_to(cp)
					if d2 < smin2:
						var dfc := half - sqrt(d2)
						if dfc > worst:
							worst = dfc
	return worst

# 真实穿模判据(严格): A 的顶点落在 B 某三角形内部投影内, 且沿 B 法向的有符号距离在
# [-half, +half] 之内 => 该顶点戳穿/夹在 B 的布面里(肉眼可见的穿模)。
# 返回: [穿模点数, 最深有符号穿透(m)]
func face_cross_count() -> Array:
	var half: float = cloths[0].cloth_thickness * 2.0 * 0.5
	var max_tri_r := 0.0
	for cl in cloths:
		max_tri_r = maxf(max_tri_r, cl.tri_r)
	# 搜索半径: 顶点扎进对层三角形时, 到最近"顶点"的距离可达外接半径 R
	var search: float = half + max_tri_r
	var inv := 1.0 / search
	_cross_build(inv)
	var search2 := search * search
	var cnt := 0
	var worst := 0.0
	# 收集候选顶点(去重), 再逐个测其关联三角形
	for g in _owner_of.size():
		var ci: int = _owner_of[g]
		var A = cloths[ci]
		var i: int = _local_of[g]
		var p: Vector3 = A.pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		for off in XPBDCloth._OFFS:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if not _xhash.has(key):
				continue
			for jj in _xhash[key]:
				var cj: int = _owner_of[jj]
				if cj == ci:
					continue
				var B = cloths[cj]
				var k: int = _local_of[jj]
				if p.distance_squared_to(B.pos[k]) > search2:
					continue
				for t in range(B.vtx_tri_ofs[k], B.vtx_tri_ofs[k + 1]):
					var tid: int = B.vtx_tri_ids[t]
					var i0: int = B.tri_verts[tid * 3]
					var i1: int = B.tri_verts[tid * 3 + 1]
					var i2: int = B.tri_verts[tid * 3 + 2]
					var cp := _closest_on_tri(p, B.pos[i0], B.pos[i1], B.pos[i2])
					if cp.distance_to(p) > half:
						continue
					var nrm: Vector3 = (B.pos[i1] - B.pos[i0]).cross(B.pos[i2] - B.pos[i0])
					var nl: float = nrm.length()
					if nl < 1e-9:
						continue
					nrm /= nl
					var sd: float = (p - cp).dot(nrm)
					# 内部投影: 最近点不在三角形边上
					var e_ab: float = _edge_dist(cp, B.pos[i0], B.pos[i1])
					var e_bc: float = _edge_dist(cp, B.pos[i1], B.pos[i2])
					var e_ca: float = _edge_dist(cp, B.pos[i2], B.pos[i0])
					if e_ab > 1e-6 and e_bc > 1e-6 and e_ca > 1e-6:
						cnt += 1
						if absf(sd) > worst:
							worst = absf(sd)
						break
	return [cnt, worst]

func _edge_dist(p: Vector3, a: Vector3, b: Vector3) -> float:
	return p.distance_to(XPBDCollider.closest_on_seg(a, b, p))

# ---------- 布布互碰: 跨布料共享空间哈希, 不同布料的顶点对等质量推开 ----------
var _xhash: Dictionary = {}
var _owner_of: PackedInt32Array = PackedInt32Array()
var _local_of: PackedInt32Array = PackedInt32Array()

func _cross_contact(push: bool) -> int:
	if cloths.size() < 2 or not enable_cloth_collision:
		return 0
	# AABB 粗筛: 任两布料包围盒(略膨胀)相交才求解 — 静置分离的布料是常态
	var reach: float = cross_min_d()
	var boxes: Array = []
	for cl in cloths:
		boxes.append(_cloth_aabb(cl))
	for i in boxes.size():
		for j in range(i + 1, boxes.size()):
			var a: AABB = boxes[i]
			var b: AABB = boxes[j]
			if a.position.x - reach <= b.position.x + b.size.x and b.position.x - reach <= a.position.x + a.size.x 			and a.position.y - reach <= b.position.y + b.size.y and b.position.y - reach <= a.position.y + a.size.y 			and a.position.z - reach <= b.position.z + b.size.z and b.position.z - reach <= a.position.z + a.size.z:
				return _cross_contact_real(push)
	return 0

func _cloth_aabb(cl) -> AABB:
	var lo: Vector3 = cl.pos[0]
	var hi: Vector3 = cl.pos[0]
	for i in cl.pos.size():
		lo = lo.min(cl.pos[i])
		hi = hi.max(cl.pos[i])
	return AABB(lo, hi - lo)

# 跨布最小间距: 纯顶点式接触的几何补偿(消除"顶点扎进对层布面"的盲区)。
# 设 A 的顶点 p 到 B 所有顶点的距离 >= D; B 的三角形外接圆半径为 R, 则三角形内部任一点
# 到其最近顶点的距离 <= R, 由三角不等式 p 到 B 表面的距离 >= D - R。
# 因此要保证"面间距 >= S"必须 D >= S + R (相加)。实测 5 层静置: 顶点口径 0.00mm,
# 却有 289 点戳穿、最深 34.9mm — 正是这个盲区。
func cross_min_d() -> float:
	# 目标是"布面不相交"(表面间距 >= S_visual), 而非"顶点间距 >= 2倍布厚"。
	# 三角不等式: d_表面 >= d_顶点 - R  =>  取 d_顶点 = S_visual + R 即可保证
	# d_表面 >= S_visual > 0 (数学上排除穿模)。
	# 反例说明为何不能取 2倍布厚: 20x20/2m 网格 R=0.0747 > 0.07, 顶点间距达标时
	# 顶点仍可落进对层三角形内部 => 实测 289 点戳穿、最深 34.9mm。
	var s: float = cloths[0].cloth_thickness * 0.5
	var r := 0.0
	for cl in cloths:
		r = maxf(r, cl.tri_r)
	return s + r

func _cross_contact_real(push: bool) -> int:
	var min_d: float = cross_min_d()
	var min_d2 := min_d * min_d
	# 哈希格 = 检测距离: 一格邻域(27 桶)即可覆盖判据, 桶内顶点数仍然很低(体积小)。
	# (若用基准 0.07 作格而判据 0.092, 就需 2 圈=125 桶扫描, 实测 6M 次字典查询/帧)
	var cell: float = min_d
	var inv: float = 1.0 / cell
	if not push:
		_cross_build(inv)
		return int(_cross_walk(inv, min_d, min_d2 * 0.25, 0))
	# 求解: 单遍扫掠存在扫内重压盲区(后续推挤把已解对压回), 实测校验驱动加扫
	# 收敛门限 0.005; 上限 3 扫(补偿后单次扫描已覆盖更严判据, 无需 8 扫)
	for sweep in 3:
		_cross_pushed = false
		_cross_build(inv)
		var worst := _cross_walk(inv, min_d, min_d2, 1)
		if worst < 0.005 and not _cross_pushed:
			break
		_cross_build(inv)
		if _cross_walk(inv, min_d, min_d2, 2) < 0.005:
			break
	return 0

var _cross_pushed := false
var _offs2: Array = []
var face_pushes: int = 0   # 面级布间推挤次数(诊断用)

# 性能关键: 只哈希"落在其它布料扩展包围盒内"的顶点。
# 默认场景两块布各占一方, 真正可能接触的顶点很少; 全量哈希+全扫描会白跑 80% 的时间
# (实测 cross=49.7ms/子步 vs solve=4.5ms)。
func _cross_build(inv: float) -> void:
	_xhash.clear()
	var margin := 1.0 / inv
	# 每块布的扩展 AABB
	var boxes: Array = []
	for cl in cloths:
		var bb: AABB = _cloth_aabb(cl)
		bb.position -= Vector3.ONE * margin
		bb.size += Vector3.ONE * (margin * 2.0)
		boxes.append(bb)
	var owner_tmp := PackedInt32Array()
	var local_tmp := PackedInt32Array()
	var gid := 0
	for ci in cloths.size():
		var cl = cloths[ci]
		for i in cl.pos.size():
			var p: Vector3 = cl.pos[i]
			var relevant := false
			for cj in cloths.size():
				if cj == ci:
					continue
				if (boxes[cj] as AABB).has_point(p):
					relevant = true
					break
			if not relevant:
				continue
			var key := (int(floor(p.x * inv)) + 512) << 20 | (int(floor(p.y * inv)) + 512) << 10 | (int(floor(p.z * inv)) + 512)
			if _xhash.has(key):
				_xhash[key].append(gid)
			else:
				_xhash[key] = [gid]
			owner_tmp.append(ci)
			local_tmp.append(i)
			gid += 1
	_owner_of = owner_tmp
	_local_of = local_tmp

# mode: 0=统计(深入min_d*0.5的对数, HUD审计) 1=求解推开(记录求解时最深欠量) 2=实测最深欠量(不推)
func _cross_walk(inv: float, min_d: float, check_d2: float, mode: int) -> float:
	var count := 0
	var worst := 0.0
	# 断言: 调用方保证 cell == min_d (即 min_d*inv == 1), 只需扫 1 圈 27 桶。
	# 若将来解耦格与判据, 必须重新评估邻域圈数, 否则会漏检(踩过: 2 圈=125 桶, 6M 次查询/帧)
	var offs: Array = XPBDCloth._OFFS
	for g in _owner_of.size():
		var ci: int = _owner_of[g]
		var cl = cloths[ci]
		var i: int = _local_of[g]
		var p: Vector3 = cl.pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		for off in offs:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if not _xhash.has(key):
				continue
			var bucket: Array = _xhash[key]
			for jj in bucket.size():
				var j: int = bucket[jj]
				if j <= g:
					continue
				var cj: int = _owner_of[j]
				if cj == ci:
					continue   # 同布料交给 self_collision
				var cl2 = cloths[cj]
				var k: int = _local_of[j]
				var d: Vector3 = p - cl2.pos[k]
				var d2 := d.length_squared()
				if d2 >= check_d2 or d2 < 1e-12:
					continue
				var wa := 0.0 if (cl.pinned[i] == 1 or (cl.grabbing and i == cl.grab_id)) else 1.0
				var wb := 0.0 if (cl2.pinned[k] == 1 or (cl2.grabbing and k == cl2.grab_id)) else 1.0
				if mode == 2:
					if wa + wb == 0.0:
						continue
					var dfc := min_d - sqrt(d2)
					if dfc > worst:
						worst = dfc
					continue
				if mode == 0:
					count += 1
					continue
				var w := wa + wb
				if w == 0.0:
					continue
				var ln := sqrt(d2)
				if min_d - ln > worst:
					worst = min_d - ln
				var nrm := d / ln
				var mv := (min_d - ln) / w
				cl.pos[i] += nrm * mv * wa
				cl2.pos[k] -= nrm * mv * wb
				p = cl.pos[i]
				_cross_pushed = true
				cl._pushed_soft[i] = 1
				cl2._pushed_soft[k] = 1
	if mode == 0:
		return float(count)
	return worst

# ---------- 渲染/HUD ----------
func _process(delta: float) -> void:
	_run_sim(minf(delta, 1.0 / 30.0))
	if _frame_count % 15 == 0:
		pen_count = 0
		var worst_self := 0.0
		for cl in cloths:
			pen_count += cl.penetration_count()
			worst_self = maxf(worst_self, cl.contact_deficit)
		if cloths.size() >= 2 and enable_cloth_collision:
			pen_inter = _cross_contact(false)
		var act := "空闲"
		if drag_collider:
			act = "拖拽碰撞体"
		elif grabbed_cloth != null and grabbed_cloth.grabbing:
			act = "抓取布料"
		if hud:
			hud.text = "FPS: %d | %s | 穿透: 物体 %d / 布间 %d | 自碰欠量 %.1fmm" % [Engine.get_frames_per_second(), act, pen_count, pen_inter, worst_self * 1000.0]
			hud.add_theme_color_override("font_color", Color(0.5, 1, 0.5) if pen_count + pen_inter == 0 else Color(1, 0.4, 0.3))
	for cl in cloths:
		cl.update_mesh()

func _update_cam() -> void:
	var cp := Vector3(
		cam_target.x + cam_dist * cos(cam_pitch) * sin(cam_yaw),
		cam_target.y - cam_dist * sin(cam_pitch),
		cam_target.z + cam_dist * cos(cam_pitch) * cos(cam_yaw))
	cam.position = cp
	cam.look_at(cam_target)

# ---------- 交互 ----------
func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_R:
				_release_all()
				_rebuild_cloths()
			KEY_Q:
				_nudge_yaw(0.18)
			KEY_E:
				_nudge_yaw(-0.18)
		return
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_LEFT:
			if mb.pressed:
				_press_left(mb.position)
			else:
				_release_all()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			cam_dist = maxf(1.5, cam_dist - 0.25)
			_update_cam()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			cam_dist = minf(8.0, cam_dist + 0.25)
			_update_cam()
	elif event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if mm.button_mask & MOUSE_BUTTON_MASK_RIGHT:
			cam_yaw -= mm.relative.x * 0.008
			cam_pitch = clampf(cam_pitch - mm.relative.y * 0.006, -1.4, 0.3)
			_update_cam()
		elif drag_collider:
			_drag_collider_to(mm.position)
		elif grabbed_cloth != null and grabbed_cloth.grabbing:
			var c := cam.project_ray_origin(mm.position)
			var dir := cam.project_ray_normal(mm.position)
			grabbed_cloth.grab_target = c + dir * grabbed_cloth.grab_depth

func _nudge_yaw(d: float) -> void:
	blob_yaw += d
	blob_root.rotation.y = blob_yaw

func _press_left(screen_pos: Vector2) -> void:
	var ro := cam.project_ray_origin(screen_pos)
	var rn := cam.project_ray_normal(screen_pos)
	# 优先: 拾取碰撞体(局部空间射线)
	var xfi: Transform3D = blob_root.global_transform.affine_inverse()
	var t: float = collider.ray_hit(xfi * ro, (xfi.basis * rn).normalized())
	if t >= 0.0:
		drag_collider = true
		blob_moving = false
		if cb_moving:
			cb_moving.set_pressed_no_signal(false)
		var hit_w := ro + rn * t
		drag_offset = hit_w - blob_center
		drag_depth = (hit_w - ro).length()
		drag_pos = blob_center
		return
	# 否则: 在所有布料中拾取射线上最近顶点
	var best_cl = null
	var best_i := -1
	var best_along := 1e9
	for cl in cloths:
		var i: int = cl.pick_vertex(ro, rn)
		if i < 0:
			continue
		var along: float = (cl.pos[i] - ro).dot(rn)
		if along < best_along:
			best_along = along
			best_cl = cl
			best_i = i
	if best_cl != null:
		grabbed_cloth = best_cl
		best_cl.start_grab(best_i, ro, rn)

func _drag_collider_to(screen_pos: Vector2) -> void:
	var ro := cam.project_ray_origin(screen_pos)
	var rn := cam.project_ray_normal(screen_pos)
	var target := ro + rn * drag_depth - drag_offset
	# Y 下限: 不允许把碰撞体压到与地面之间形成 < 2倍布厚的缝 — 布料被夹进亚布厚缝隙
	# 会被反复揉搓成永久褶皱网络(解不出), 这是"压扁成一团"的根源
	var half_y := 0.45
	if collider_mode == 1:
		half_y = 0.88
	elif collider_mode == 2:
		half_y = 0.30
	var y_min: float = ground_y + half_y + cloth_thickness * 2.5
	drag_pos = Vector3(
		clampf(target.x, -3.0, 3.0),
		clampf(target.y, maxf(-1.25, y_min), 2.6),
		clampf(target.z, -3.0, 3.0))

func _release_all() -> void:
	if drag_collider:
		drag_collider = false
	for cl in cloths:
		if cl.grabbing:
			cl.release_grab(substeps)
	grabbed_cloth = null

# ---------- UI ----------
var hud: Label
var cb_moving: CheckButton
var _sliders := {}

func _build_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var panel := PanelContainer.new()
	panel.position = Vector2(10, 10)
	panel.custom_minimum_size = Vector2(240, 0)
	layer.add_child(panel)
	var vbox := VBoxContainer.new()
	panel.add_child(vbox)
	var title := Label.new()
	title.text = "XPBD 多布料 (R=重置)"
	vbox.add_child(title)
	var tip := Label.new()
	tip.text = "左键点碰撞体=拖动(Q/E旋转)\n左键点布料=抓取甩动 | 右键=相机"
	tip.add_theme_font_size_override("font_size", 11)
	tip.add_theme_color_override("font_color", Color(0.7, 0.75, 0.8))
	vbox.add_child(tip)
	hud = Label.new()
	hud.text = "FPS: -- | 空闲"
	hud.add_theme_color_override("font_color", Color(0.5, 1, 0.5))
	vbox.add_child(hud)
	_add_slider(vbox, "柔软度", 0.0, 1.0, softness, func(v: float) -> void:
		softness = v
		_sync_params())
	_add_slider(vbox, "风力", 0.0, 20.0, wind_strength, func(v: float) -> void: wind_strength = v)
	_add_slider(vbox, "空气阻力", 0.0, 0.1, air_drag, func(v: float) -> void:
		air_drag = v
		_sync_params())
	_add_slider(vbox, "摩擦力", 0.0, 1.0, friction, func(v: float) -> void:
		friction = v
		_sync_params())
	_add_slider(vbox, "布厚(自/互碰)", 0.02, 0.08, cloth_thickness, func(v: float) -> void:
		cloth_thickness = v
		_sync_params())
	_add_slider(vbox, "抗拉伸", 1.02, 1.3, max_stretch, func(v: float) -> void:
		max_stretch = v
		_sync_params())
	_add_slider(vbox, "子步数", 2, 10, substeps, func(v: float) -> void: substeps = maxi(1, int(v)))
	_add_slider(vbox, "抓取力度", 0.2, 1.0, grab_stiffness, func(v: float) -> void:
		grab_stiffness = v
		_sync_params())
	_add_slider(vbox, "拖拽限速", 1.0, 6.0, drag_speed, func(v: float) -> void: drag_speed = v)
	_add_mode_row(vbox, "碰撞体", [["岩石", 0], ["胶囊", 1], ["方箱", 2]], func(m: int) -> void:
		collider_mode = m
		_release_all()
		_build_blob())
	_add_mode_row(vbox, "布料数量", [["1", 1], ["2", 2], ["3", 3], ["4", 4], ["5", 5]], func(m: int) -> void:
		cloth_count = m
		_release_all()
		_rebuild_cloths())
	var row3 := HBoxContainer.new()
	var lab3 := Label.new()
	lab3.text = "物体运动"
	lab3.custom_minimum_size = Vector2(90, 0)
	row3.add_child(lab3)
	cb_moving = CheckButton.new()
	cb_moving.button_pressed = blob_moving
	cb_moving.toggled.connect(func(on: bool) -> void: blob_moving = on)
	row3.add_child(cb_moving)
	vbox.add_child(row3)

func _add_mode_row(parent: VBoxContainer, label: String, items: Array, cb: Callable) -> void:
	var row := HBoxContainer.new()
	var lab := Label.new()
	lab.text = label
	lab.custom_minimum_size = Vector2(90, 0)
	row.add_child(lab)
	for it in items:
		var b := Button.new()
		b.text = it[0]
		b.toggle_mode = true
		b.pressed.connect(func() -> void:
			cb.call(it[1])
			for c in row.get_children():
				if c is Button and c != b:
					c.button_pressed = false)
		row.add_child(b)
	parent.add_child(row)

func _add_slider(parent: VBoxContainer, label: String, mn: float, mx: float, val: float, cb: Callable) -> void:
	var row := HBoxContainer.new()
	var lab := Label.new()
	lab.text = label
	lab.custom_minimum_size = Vector2(90, 0)
	row.add_child(lab)
	var sl := HSlider.new()
	sl.min_value = mn
	sl.max_value = mx
	sl.step = (mx - mn) / 100.0
	sl.value = val
	sl.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	sl.custom_minimum_size = Vector2(120, 16)
	sl.value_changed.connect(cb)
	row.add_child(sl)
	parent.add_child(row)
	_sliders[label] = sl
