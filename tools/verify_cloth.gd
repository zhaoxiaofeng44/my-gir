extends SceneTree
# XPBD 多布料完整压测矩阵 v2 (全顶点精确审计)
# A 物体碰撞: A1落体 A2抛掷45/120 A3拖拽穿布 A4运动碰撞体 A5旋转扫掠 A6抓布硬拽过碰撞体 A7砸地
# B 自碰撞:   B1对折挤压 B2揉团 (自穿透精确审计: 排除网格邻接, 深入 min_d*0.5 判穿)
# C 布布碰撞: C1堆叠静置 C2高速穿帘 C3拖拽压双布 C4抓甩穿帘 (跨布精确审计, 排除抓取点)
# D 参数极端: D1 substeps=2+薄布0.02+零摩擦+抛掷  D2 substeps=10 稳定
# E 稳定性:   E1 长时风载静置 (NaN/速度爆炸/穿透)
# 用法: godot --headless --path . -s tools/verify_cloth.gd

var scene: Node
var fail := 0
var cases := 0

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)   # 手动步进, 全速压测
	# CLOTH_VERIFY 环境变量: 逗号分隔组别(A/B/C/D/E/F), 缺省全跑(分块供CI/超时分段)
	var groups: String = OS.get_environment("CLOTH_VERIFY")
	var want := {}
	for g in groups.split(",", false):
		want[g.strip_edges().to_upper()] = true
	var has := func(g: String) -> bool: return want.is_empty() or want.has(g)
	if has.call("A"):
		for m in 3:
			_t_freefall(m)
			_t_throw(m)
			_t_drag(m)
			_t_moving(m)
		_t_spin()
		_t_grab_yank_obj()
		_t_ground_slam()
	if has.call("B"):
		_t_fold()
		_t_crumple()
	if has.call("C"):
		_t_pile()
		_t_throw_curtain()
		_t_drag_squeeze()
		_t_grab_yank_inter()
	if has.call("D"):
		_t_extreme_low()
		_t_extreme_sub10()
	if has.call("E"):
		_t_longrun()
	if has.call("F"):
		_t_cover5()
	print("========== 结论: %d/%d PASS%s ==========" % [cases - fail, cases, "" if fail == 0 else "  (%d FAIL)" % fail])
	quit()

# ---------- 公共 ----------
func _setup(mode: int, ncloth: int = 2) -> void:
	scene.collider_mode = mode
	scene.blob_moving = false
	scene.drag_collider = false
	scene.blob_yaw = 0.0
	scene.blob_center = Vector3(0, 0.2, 0)
	scene.wind_strength = 0.0
	scene.substeps = 4
	scene.cloth_thickness = 0.035
	scene.friction = 0.85
	scene.softness = 0.5
	scene.cloth_count = ncloth
	scene._build_blob()
	scene._rebuild_cloths()
	scene.blob_root.force_update_transform()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)

func _ok(tag: String, ok: bool, detail: String) -> void:
	cases += 1
	if not ok:
		fail += 1
	print("  [%s] %s  %s" % ["PASS" if ok else "FAIL", tag, detail])

func _fmt(x: float) -> String:
	return "%.4f" % x

# ---------- 审计 (全顶点精确) ----------
# 物体穿透: 顶点深入碰撞体视觉面(pad*0.5 容差)的最大深度
func _worst_pen() -> float:
	var worst := 0.0
	var col: XPBDCollider = scene.collider
	var xfi: Transform3D = col.root.global_transform.affine_inverse()
	for cl in scene.cloths:
		var pad: float = cl.cloth_thickness * 0.5 + cl.collision_margin
		for i in range(cl.pos.size()):
			var p: Vector3 = xfi * cl.pos[i]
			for k in col.count:
				var depth := 0.0
				var ty: int = col.pt[k]
				if ty == 0:
					depth = col.pr[k] + pad * 0.5 - (p - col.poff[k]).length()
				elif ty == 1:
					depth = col.pr[k] + pad * 0.5 - (p - XPBDCollider.closest_on_seg(col.pa[k], col.pb[k], p)).length()
				else:
					var l: Vector3 = p - col.poff[k]
					var h: Vector3 = col.phalf[k]
					if absf(l.x) < h.x and absf(l.y) < h.y and absf(l.z) < h.z:
						depth = minf(minf(h.x - absf(l.x), h.y - absf(l.y)), h.z - absf(l.z)) + pad * 0.5
				if depth > worst:
					worst = depth
	return worst

const _OFFS: Array = [Vector3i(-1,-1,-1),Vector3i(-1,-1,0),Vector3i(-1,-1,1),Vector3i(-1,0,-1),Vector3i(-1,0,0),Vector3i(-1,0,1),Vector3i(-1,1,-1),Vector3i(-1,1,0),Vector3i(-1,1,1),Vector3i(0,-1,-1),Vector3i(0,-1,0),Vector3i(0,-1,1),Vector3i(0,0,-1),Vector3i(0,0,1),Vector3i(0,1,-1),Vector3i(0,1,0),Vector3i(0,1,1),Vector3i(1,-1,-1),Vector3i(1,-1,0),Vector3i(1,-1,1),Vector3i(1,0,-1),Vector3i(1,0,0),Vector3i(1,0,1),Vector3i(1,1,-1),Vector3i(1,1,0),Vector3i(1,1,1)]

func _hash_key(p: Vector3, inv: float) -> int:
	return (int(floor(p.x * inv)) + 512) << 20 | (int(floor(p.y * inv)) + 512) << 10 | (int(floor(p.z * inv)) + 512)

# 哈希构建: 返回 hash + owner_of/local_of (全局id -> 布索引/局部顶点id)
func _hash_build(cloths: Array, cell: float) -> Array:
	var hh := {}
	var owner_of := PackedInt32Array()
	var local_of := PackedInt32Array()
	var inv := 1.0 / cell
	var gid := 0
	for ci in cloths.size():
		var cl: XPBDCloth = cloths[ci]
		for i in cl.pos.size():
			var key: int = _hash_key(cl.pos[i], inv)
			if hh.has(key):
				hh[key].append(gid)
			else:
				hh[key] = [gid]
			owner_of.append(ci)
			local_of.append(i)
			gid += 1
	return [hh, owner_of, local_of]

# 抓取点豁免
func _is_forced(cl: XPBDCloth, i: int) -> bool:
	return cl.grabbing and i == cl.grab_id

# 布间穿透: 不同布料顶点对深入 min_d*0.5 的最大深度(排除抓取点: 用户强制力豁免)
func _worst_inter_exact() -> float:
	var cloths: Array = scene.cloths
	if cloths.size() < 2:
		return 0.0
	var min_d: float = cloths[0].cloth_thickness * 2.0
	var built: Array = _hash_build(cloths, min_d)
	var hh: Dictionary = built[0]
	var owner_of: PackedInt32Array = built[1]
	var local_of: PackedInt32Array = built[2]
	var inv := 1.0 / min_d
	var worst := 0.0
	var total: int = owner_of.size()
	for g in total:
		var ci: int = owner_of[g]
		var i: int = local_of[g]
		var cl: XPBDCloth = cloths[ci]
		if _is_forced(cl, i):
			continue
		var p: Vector3 = cl.pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		for off in _OFFS:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if not hh.has(key):
				continue
			for gid in hh[key]:
				if gid <= g:
					continue
				var cj: int = owner_of[gid]
				if cj == ci:
					continue
				var k: int = local_of[gid]
				if _is_forced(cloths[cj], k):
					continue
				var d: float = p.distance_to(cloths[cj].pos[k])
				if d < min_d * 0.5 and (min_d * 0.5 - d) > worst:
					worst = min_d * 0.5 - d
	return worst

# 自碰撞穿透: 单布顶点对(排除网格邻接 dr<=2且dc<=2 与抓取点)深入 min_d*0.5 的最大深度
func _worst_self(cl: XPBDCloth) -> float:
	var min_d: float = cl.cloth_thickness * 2.0
	var inv := 1.0 / min_d
	var hh := {}
	for i in cl.pos.size():
		var key: int = _hash_key(cl.pos[i], inv)
		if hh.has(key):
			hh[key].append(i)
		else:
			hh[key] = [i]
	var worst := 0.0
	var cols: int = cl.cols
	for i in cl.pos.size():
		if _is_forced(cl, i):
			continue
		var p: Vector3 = cl.pos[i]
		var cx := int(floor(p.x * inv)) + 512
		var cy := int(floor(p.y * inv)) + 512
		var cz := int(floor(p.z * inv)) + 512
		var ri := i / cols
		var ci2 := i % cols
		for off in _OFFS:
			var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
			if not hh.has(key):
				continue
			for j in hh[key]:
				if j <= i:
					continue
				var dr: int = absi(j / cols - ri)
				var dc: int = absi(j % cols - ci2)
				if dr <= 2 and dc <= 2:
					continue
				var d: float = p.distance_to(cl.pos[j])
				if d < min_d * 0.5 and (min_d * 0.5 - d) > worst:
					worst = min_d * 0.5 - d
	return worst

func _bounds_ok() -> bool:
	for cl in scene.cloths:
		for i in range(cl.pos.size()):
			if not cl.pos[i].is_finite() or not cl.vel[i].is_finite():
				return false
			if cl.pos[i].y < cl.ground_y - 0.002:
				return false
	return true

func _max_speed() -> float:
	var mx := 0.0
	for cl in scene.cloths:
		for i in cl.vel.size():
			mx = maxf(mx, cl.vel[i].length())
	return mx

# ---------- A 物体碰撞 ----------
func _t_freefall(m: int) -> void:
	_setup(m, 2)
	_run(300)
	var pen := 0.0
	var inter := 0.0
	for _c in 5:
		pen = maxf(pen, _worst_pen())
		inter = maxf(inter, _worst_inter_exact())
		_run(12)
	print("--- A1 模式%d 自由落体 ---" % m)
	_ok("A1 模式%d 落体/物体" % m, pen <= 0.0, "最大穿透=" + _fmt(pen))
	_ok("A1 模式%d 落体/布间" % m, inter <= 0.0, "最大穿透=" + _fmt(inter))
	_ok("A1 模式%d 落体/边界" % m, _bounds_ok(), "NaN/地面")

func _t_throw(m: int) -> void:
	_setup(m, 2)
	_run(50)
	var worst := 0.0
	for speed in [45.0, 120.0]:
		var cl: XPBDCloth = scene.cloths[0]
		for i in cl.pos.size():
			var dir: Vector3 = (Vector3(0, 0.2, 0) - cl.pos[i]).normalized()
			cl.vel[i] = dir * speed
		for _s in 90:
			_run(1)
			worst = maxf(worst, _worst_pen())
	print("--- A2 模式%d 高速抛掷(45/120 m/s, 逐步审计) ---" % m)
	_ok("A2 模式%d 抛掷" % m, worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A2 模式%d 抛掷/边界" % m, _bounds_ok(), "NaN/地面")

func _t_drag(m: int) -> void:
	_setup(m, 3)
	_run(150)
	var worst := 0.0
	var worst_i := 0.0
	var paths := [
		[Vector3(-1.6, 1.0, -0.6), Vector3(1.6, 1.0, -0.6)],
		[Vector3(0, 2.2, -1.0), Vector3(0, -0.6, 1.0)],
	]
	scene.drag_collider = true
	for path in paths:
		scene.drag_pos = path[1]
		for s in 110:
			if s % 12 == 0:
				scene.blob_yaw += 0.12
				scene.blob_root.rotation.y = scene.blob_yaw
			_run(1)
			worst = maxf(worst, _worst_pen())
			if s % 2 == 0:
				worst_i = maxf(worst_i, _worst_inter_exact())
	scene.drag_collider = false
	print("--- A3 模式%d 拖拽穿布(含布间审计) ---" % m)
	_ok("A3 模式%d 拖拽/物体" % m, worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A3 模式%d 拖拽/布间" % m, worst_i <= 0.0, "最大穿透=" + _fmt(worst_i))
	_ok("A3 模式%d 拖拽/边界" % m, _bounds_ok(), "NaN/地面")

func _t_moving(m: int) -> void:
	_setup(m, 2)
	_run(60)
	scene.blob_moving = true
	var worst := 0.0
	for s in 200:
		_run(1)
		if s % 4 == 0:
			worst = maxf(worst, _worst_pen())
	scene.blob_moving = false
	print("--- A4 模式%d 运动碰撞体 ---" % m)
	_ok("A4 模式%d 运动" % m, worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A4 模式%d 运动/边界" % m, _bounds_ok(), "NaN/地面")

# 旋转扫掠: 碰撞体半嵌进挂帘平面内高速自转
func _t_spin() -> void:
	_setup(0, 2)
	scene.blob_center = Vector3(-1.0, 1.1, -0.6)
	scene.blob_root.position = scene.blob_center
	_run(80)
	var worst := 0.0
	for s in 50:
		scene.blob_yaw += 0.15
		scene.blob_root.rotation.y = scene.blob_yaw
		_run(1)
		worst = maxf(worst, _worst_pen())
	print("--- A5 旋转扫掠(0.15 rad/步 x50) ---")
	_ok("A5 旋转", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A5 旋转/边界", _bounds_ok(), "NaN/地面")

# 抓布硬拽: 抓取刚度1.0, 目标点穿过岩石内部
func _t_grab_yank_obj() -> void:
	_setup(0, 2)
	_run(120)
	var cl: XPBDCloth = scene.cloths[0]
	cl.grab_stiffness = 1.0
	cl.grabbing = true
	cl.grab_id = (cl.rows / 2) * cl.cols + cl.cols / 2
	var from := Vector3(0, 1.0, 0)
	var to := Vector3(0, -0.8, 1.4)
	var worst := 0.0
	for s in 50:
		cl.grab_target = from.lerp(to, float(s + 1) / 50.0)
		_run(1)
		worst = maxf(worst, _worst_pen())
	cl.release_grab(scene.substeps)
	print("--- A6 抓布硬拽过碰撞体(刚度1.0) ---")
	_ok("A6 硬拽/物体", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A6 硬拽/边界", _bounds_ok(), "NaN/地面")

func _t_ground_slam() -> void:
	_setup(0, 2)
	_run(40)
	var cl: XPBDCloth = scene.cloths[0]
	for i in cl.pos.size():
		cl.vel[i] = Vector3(0, -30, 0)
	var worst := 0.0
	var ok := true
	for s in 100:
		_run(1)
		worst = maxf(worst, _worst_pen())
		if s % 10 == 0:
			ok = ok and _bounds_ok()
	print("--- A7 30m/s 砸地 ---")
	_ok("A7 砸地/物体", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("A7 砸地/边界", ok and _bounds_ok(), "NaN/地面")

# ---------- B 自碰撞 ----------
# 对折挤压: 布落地面后左右两半相向压
func _t_fold() -> void:
	_setup(0, 1)
	scene.blob_center = Vector3(0, -1.5, 0)
	scene.blob_root.position = scene.blob_center
	_run(120)
	var cl: XPBDCloth = scene.cloths[0]
	var center := Vector3(0, cl.ground_y + 0.15, 0)
	var worst := 0.0
	for s in 50:
		for i in cl.pos.size():
			var dir: Vector3 = center - cl.pos[i]
			dir.y = 0.0
			cl.vel[i] = dir.normalized() * 3.0 if dir.length() > 0.05 else Vector3.ZERO
		_run(1)
		if s % 5 == 0:
			worst = maxf(worst, _worst_self(cl))
	var settle := 0.0
	for _c in 10:
		_run(15)
		settle = maxf(settle, _worst_self(cl))
	print("--- B1 对折挤压 ---")
	_ok("B1 对折/自穿透(挤压中,≤30mm)", worst <= 0.03, "峰值=" + _fmt(worst))
	_ok("B1 对折/自愈(松手后)", settle <= 0.0, "残余=" + _fmt(settle))
	_ok("B1 对折/物体", _worst_pen() <= 0.0, "最大穿透=" + _fmt(_worst_pen()))
	_ok("B1 对折/边界", _bounds_ok(), "NaN/地面")

# 揉团: 全部顶点强力向心, 揉成球
func _t_crumple() -> void:
	_setup(0, 1)
	scene.blob_center = Vector3(0, -1.5, 0)
	scene.blob_root.position = scene.blob_center
	_run(100)
	var cl: XPBDCloth = scene.cloths[0]
	var center := Vector3(0, cl.ground_y + 0.35, 0)
	var worst := 0.0
	for s in 60:
		for i in cl.pos.size():
			var dir: Vector3 = center - cl.pos[i]
			cl.vel[i] = dir.normalized() * 5.0 if dir.length() > 0.05 else Vector3.ZERO
		_run(1)
		if s % 5 == 0:
			worst = maxf(worst, _worst_self(cl))
	# 强制5m/s向心挤压中的瞬态(≤30mm)可接受; 松手后必须自愈
	var settle := 0.0
	for _c in 10:
		_run(15)
		settle = maxf(settle, _worst_self(cl))
	print("--- B2 揉团 ---")
	_ok("B2 揉团/自穿透(挤压中,≤30mm)", worst <= 0.03, "峰值=" + _fmt(worst))
	_ok("B2 揉团/自愈(松手后)", settle <= 0.0, "残余=" + _fmt(settle))
	_ok("B2 揉团/边界", _bounds_ok(), "NaN/地面")

# ---------- C 布布碰撞 ----------
func _t_pile() -> void:
	_setup(0, 3)
	_run(400)
	var worst := 0.0
	for _c in 5:
		worst = maxf(worst, _worst_inter_exact())
		_run(20)
	print("--- C1 3布料堆叠静置 ---")
	_ok("C1 堆叠/布间", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("C1 堆叠/物体", _worst_pen() <= 0.0, "最大穿透=" + _fmt(_worst_pen()))
	_ok("C1 堆叠/边界", _bounds_ok(), "NaN/地面")

# 高速穿帘: 落体布 15m/s 射向挂帘(逐步跨布审计, 探测隧穿)
func _t_throw_curtain() -> void:
	_setup(0, 2)
	_run(150)
	var cl: XPBDCloth = scene.cloths[0]
	var target := Vector3(-1.35, 1.2, -0.6)
	for i in cl.pos.size():
		var dir: Vector3 = target - cl.pos[i]
		cl.vel[i] = dir.normalized() * 15.0
	var worst := 0.0
	for s in 80:
		_run(1)
		worst = maxf(worst, _worst_inter_exact())
	# 15m/s 飞行中的瞬态(≤30mm)可接受(单帧位移 > 布厚, 接触网络来不及完全分离);
	# 稳定后必须归零 — 这才是"没穿过去"的判据
	var settle := 0.0
	for _c in 6:
		_run(15)
		settle = maxf(settle, _worst_inter_exact())
	print("--- C2 高速穿帘(15 m/s, 逐步审计) ---")
	_ok("C2 穿帘/布间瞬态(飞行中,≤30mm)", worst <= 0.03, "峰值=" + _fmt(worst))
	_ok("C2 穿帘/布间自愈(稳定后)", settle <= 0.0, "残余=" + _fmt(settle))
	_ok("C2 穿帘/边界", _bounds_ok(), "NaN/地面")

# 拖拽方箱穿过落体布堆再压进挂帘(双审计)
func _t_drag_squeeze() -> void:
	_setup(2, 3)
	_run(150)
	var worst := 0.0
	var worst_i := 0.0
	scene.drag_collider = true
	scene.drag_pos = Vector3(-1.2, 0.9, -0.6)
	for s in 130:
		_run(1)
		worst = maxf(worst, _worst_pen())
		if s % 2 == 0:
			worst_i = maxf(worst_i, _worst_inter_exact())
	scene.drag_collider = false
	# 拖拽挤压中的瞬态(≤30mm)可接受; 松手后必须自愈
	var settle_i := 0.0
	for _c in 6:
		_run(15)
		settle_i = maxf(settle_i, _worst_inter_exact())
	print("--- C3 拖拽压双布(方箱穿布堆入挂帘) ---")
	_ok("C3 压双布/物体", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("C3 压双布/布间瞬态(拖拽中,≤30mm)", worst_i <= 0.03, "峰值=" + _fmt(worst_i))
	_ok("C3 压双布/布间自愈(松手后)", settle_i <= 0.0, "残余=" + _fmt(settle_i))
	_ok("C3 压双布/边界", _bounds_ok(), "NaN/地面")

# 抓布甩向挂帘并硬拽穿过(排除抓取点的跨布审计)
func _t_grab_yank_inter() -> void:
	_setup(0, 2)
	_run(150)
	var cl: XPBDCloth = scene.cloths[0]
	cl.grab_stiffness = 1.0
	cl.grabbing = true
	cl.grab_id = (cl.rows / 2) * cl.cols + cl.cols / 2
	var from := Vector3(0, 0.5, 0)
	var to := Vector3(-1.7, 1.1, -0.6)
	var worst := 0.0
	for s in 60:
		cl.grab_target = from.lerp(to, float(s + 1) / 60.0)
		_run(1)
		worst = maxf(worst, _worst_inter_exact())
	cl.release_grab(scene.substeps)
	var tail := 0.0
	for _c in 4:
		_run(15)
		tail = maxf(tail, _worst_inter_exact())
	print("--- C4 抓甩穿帘(刚度1.0, 抓取点豁免) ---")
	_ok("C4 抓甩/布间瞬态(甩动中,≤30mm)", worst <= 0.03, "峰值=" + _fmt(worst))
	_ok("C4 抓甩/布间自愈(回稳后)", tail <= 0.0, "残余=" + _fmt(tail))
	_ok("C4 抓甩/边界", _bounds_ok(), "NaN/地面")

# ---------- D 参数极端 ----------
func _t_extreme_low() -> void:
	_setup(2, 2)
	scene.substeps = 2
	scene.cloth_thickness = 0.02
	scene.friction = 0.0
	scene._sync_params()
	_run(250)
	var cl: XPBDCloth = scene.cloths[0]
	for i in cl.pos.size():
		var dir: Vector3 = (Vector3(0, 0.2, 0) - cl.pos[i]).normalized()
		cl.vel[i] = dir * 45.0
	var worst := 0.0
	for s in 90:
		_run(1)
		worst = maxf(worst, _worst_pen())
	print("--- D1 substeps=2 + 布厚0.02 + 零摩擦 + 抛掷45 ---")
	_ok("D1 极端/物体", worst <= 0.0, "最大穿透=" + _fmt(worst))
	_ok("D1 极端/边界", _bounds_ok(), "NaN/地面")

func _t_extreme_sub10() -> void:
	_setup(0, 2)
	scene.substeps = 10
	_run(200)
	print("--- D2 substeps=10 稳定性 ---")
	_ok("D2 sub10/物体", _worst_pen() <= 0.0, "最大穿透=" + _fmt(_worst_pen()))
	_ok("D2 sub10/边界", _bounds_ok(), "NaN/地面")
	_ok("D2 sub10/速度收敛", _max_speed() < 30.0, "最大速度=%.2f m/s" % _max_speed())

# ---------- E 稳定性 ----------
func _t_longrun() -> void:
	_setup(0, 2)
	scene.wind_strength = 6.0
	var pen := 0.0
	var inter := 0.0
	var ok := true
	for s in 90:
		_run(10)
		pen = maxf(pen, _worst_pen())
		inter = maxf(inter, _worst_inter_exact())
		ok = ok and _bounds_ok()
	print("--- E1 长时风载静置(900步, 风力6) ---")
	_ok("E1 长跑/物体", pen <= 0.0, "最大穿透=" + _fmt(pen))
	_ok("E1 长跑/布间", inter <= 0.0, "最大穿透=" + _fmt(inter))
	_ok("E1 长跑/边界", ok and _bounds_ok(), "NaN/地面")
	_ok("E1 长跑/速度收敛", _max_speed() < 30.0, "最大速度=%.2f m/s" % _max_speed())

# ---------- F 多布同盖一个碰撞体 ----------
func _t_cover5() -> void:
	# F1: UI真实配置(落布+挂帘x2+盖布x2), 三层布叠落岩石
	_setup(0, 5)
	_run(520)
	var worst_i := 0.0
	var worst_s := 0.0
	for _c in 5:
		worst_i = maxf(worst_i, _worst_inter_exact())
		for cl in scene.cloths:
			worst_s = maxf(worst_s, _worst_self(cl))
		_run(20)
	print("--- F1 5布叠落岩石(3层同盖+2挂帘) ---")
	_ok("F1 叠落/布间", worst_i <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_i))
	_ok("F1 叠落/自碰", worst_s <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_s))

	# F2: 限速拖拽岩石从布堆底下穿进穿出
	scene.drag_collider = true
	var worst_d := 0.0
	var worst_di := 0.0
	for target: Vector3 in [Vector3(2.4, 0.25, 0), Vector3(-2.4, 0.35, 0.6), Vector3(0, 0.2, 0)]:
		scene.drag_pos = target
		for _s in 130:
			_run(1)
			worst_d = maxf(worst_d, _worst_pen())
			worst_di = maxf(worst_di, _worst_inter_exact())
	_run(120)
	scene.drag_collider = false
	print("--- F2 拖拽岩石穿过布堆 ---")
	_ok("F2 拖拽/物体", worst_d <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_d))
	_ok("F2 拖拽/布间", worst_di <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_di))

	# F3: 极限压力学 - 5块布全部同盖岩石, 逐层堆叠
	_setup(0, 5)
	var offs: Array = [Vector3(0, 1.8, 0), Vector3(0.15, 2.5, -0.1), Vector3(-0.12, 3.1, 0.12), Vector3(0.1, 3.7, -0.05), Vector3(-0.05, 4.3, 0.1)]
	for k in 5:
		var cl: XPBDCloth = scene.cloths[k]
		cl.vertical = false
		cl.yaw = 0.0
		cl.cols = 20
		cl.rows = 20
		cl.cloth_size = Vector2(2.0 - 0.08 * k, 2.0 - 0.08 * k)
		cl.spawn_origin = offs[k]
		cl.build()
	_run(650)
	var worst_f3i := 0.0
	var worst_f3s := 0.0
	for _c in 5:
		worst_f3i = maxf(worst_f3i, _worst_inter_exact())
		for cl in scene.cloths:
			worst_f3s = maxf(worst_f3s, _worst_self(cl))
		_run(20)
	print("--- F3 5布逐层同盖岩石 ---")
	_ok("F3 堆叠/布间", worst_f3i <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_f3i))
	_ok("F3 堆叠/自碰", worst_f3s <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(worst_f3s))
	_ok("F3 堆叠/物体", _worst_pen() <= 0.0 and _bounds_ok(), "最大穿透=" + _fmt(_worst_pen()))
