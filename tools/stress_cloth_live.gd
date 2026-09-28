extends SceneTree
# 交互级压力复现 + 严格全顶点 O(n^2) 审计(比 HUD 每3采样更严)
# 复现: 静置 -> 运动碰撞体 -> 拖拽碰撞体穿布(从上砸下) -> 抓布硬甩过碰撞体
# 审计: 物体穿透 / 布间穿透 / 自碰穿透(全对扫描) / NaN / 速度爆炸
# 用法: godot --headless --path . -s tools/stress_cloth_live.gd

var scene: Node
var fail := 0
var cases := 0

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	scene.use_native_backend = true   # 回归覆盖 C++ 后端

	_t_live_stress(0)
	_t_live_stress(1)
	_t_live_stress(2)
	_t_live_stress_grab()
	print("========== 结论: %d/%d PASS%s ==========" % [cases - fail, cases, "" if fail == 0 else "  (%d FAIL)" % fail])
	quit()

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

# ---------- 严格审计 ----------
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

func _bounds_ok() -> bool:
	for cl in scene.cloths:
		for i in range(cl.pos.size()):
			if not cl.pos[i].is_finite() or not cl.vel[i].is_finite():
				return false
			if cl.pos[i].y < cl.ground_y - 0.002:
				return false
	return true

# 自碰: 单布全对 O(n^2) — 比哈希审计更严(任何位置的对都被检查)
func _worst_self_strict(cl) -> float:
	var min_d: float = cl.cloth_thickness * 2.0
	var worst := 0.0
	var n: int = cl.pos.size()
	var cols: int = cl.cols
	for i in n:
		if cl.grabbing and i == cl.grab_id:
			continue
		var p: Vector3 = cl.pos[i]
		var ri := i / cols
		var cix := i % cols
		for j in range(i + 1, n):
			var dr: int = absi(j / cols - ri)
			var dc: int = absi(j % cols - cix)
			if dr <= 2 and dc <= 2:
				continue
			var d: float = p.distance_to(cl.pos[j])
			if d < min_d * 0.5:
				var pen := min_d * 0.5 - d
				if pen > worst:
					worst = pen
	return worst

# 布间: 全对
func _worst_inter_strict() -> float:
	if scene.cloths.size() < 2:
		return 0.0
	var min_d: float = scene.cloths[0].cloth_thickness * 2.0
	var worst := 0.0
	var a = scene.cloths[0]
	var b = scene.cloths[1]
	for i in a.pos.size():
		if a.grabbing and i == a.grab_id:
			continue
		var p: Vector3 = a.pos[i]
		for j in b.pos.size():
			if b.grabbing and j == b.grab_id:
				continue
			var d: float = p.distance_to(b.pos[j])
			if d < min_d * 0.5:
				var pen := min_d * 0.5 - d
				if pen > worst:
					worst = pen
	return worst

# ---------- 压力用例 ----------
func _t_live_stress(m: int) -> void:
	_setup(m, 2)
	_run(200)   # 静置落体
	var worst_obj := 0.0
	var dyn_self := 0.0     # 主动挤压过程中的瞬态(折皱被实时压拢, 松手后应自愈)
	var dyn_inter := 0.0
	# 1) 运动碰撞体 6秒
	scene.blob_moving = true
	for s in 120:
		_run(2)
		worst_obj = maxf(worst_obj, _worst_pen())
		dyn_self = maxf(dyn_self, _worst_self_strict(scene.cloths[0]))
	scene.blob_moving = false
	# 2) 拖拽碰撞体从高处砸穿布料
	scene.drag_collider = true
	scene.drag_speed = 6.0
	scene.drag_pos = Vector3(0, 2.0, 0)
	_run(60)
	scene.drag_pos = Vector3(0, -1.2, 0)
	for s in 60:
		_run(1)
		worst_obj = maxf(worst_obj, _worst_pen())
		dyn_self = maxf(dyn_self, _worst_self_strict(scene.cloths[0]))
		dyn_inter = maxf(dyn_inter, _worst_inter_strict())
	scene.drag_collider = false
	# 3) 回稳(90步): 瞬态必须完全自愈 — 这是"不成团"的硬标准
	_run(90)
	var settle_self := _worst_self_strict(scene.cloths[0])
	var settle_inter := _worst_inter_strict()
	worst_obj = maxf(worst_obj, _worst_pen())
	print("--- S%d 模式%d 交互级压力(运动+砸穿+回稳) ---" % [m + 1, m])
	_ok("S%d 物体穿透" % (m + 1), worst_obj <= 0.0, "最大穿透=" + _fmt(worst_obj))
	_ok("S%d 自碰瞬态(挤压中,≤30mm)" % (m + 1), dyn_self <= 0.03, "峰值=" + _fmt(dyn_self))
	_ok("S%d 自碰自愈(回稳后)" % (m + 1), settle_self <= 0.0, "残余=" + _fmt(settle_self))
	_ok("S%d 布间瞬态(挤压中,≤30mm)" % (m + 1), dyn_inter <= 0.03, "峰值=" + _fmt(dyn_inter))
	_ok("S%d 布间自愈(回稳后)" % (m + 1), settle_inter <= 0.0, "残余=" + _fmt(settle_inter))
	_ok("S%d 边界" % (m + 1), _bounds_ok(), "NaN/地面")

func _t_live_stress_grab() -> void:
	_setup(0, 2)
	_run(150)
	var cl = scene.cloths[0]
	cl.grab_stiffness = 1.0
	cl.grabbing = true
	cl.grab_id = (cl.rows / 2) * cl.cols + cl.cols / 2
	var worst_obj := 0.0
	var worst_self := 0.0
	var worst_inter := 0.0
	# 甩动: 4 个方向硬拽(绕过碰撞体)
	var targets := [Vector3(0.9, 0.8, 0.6), Vector3(-0.9, 0.6, -0.7), Vector3(0.5, 0.1, 0.9), Vector3(-0.6, 0.9, -0.5)]
	for t in targets:
		for s in 30:
			cl.grab_target = cl.grab_target.lerp(t, 0.12)
			_run(1)
			worst_obj = maxf(worst_obj, _worst_pen())
		worst_self = maxf(worst_self, _worst_self_strict(cl))
		worst_inter = maxf(worst_inter, _worst_inter_strict())
	cl.release_grab(scene.substeps)
	# 回稳90步: 瞬态必须完全自愈
	_run(90)
	var settle_self := _worst_self_strict(cl)
	var settle_inter := _worst_inter_strict()
	worst_obj = maxf(worst_obj, _worst_pen())
	print("--- S4 抓布硬甩过碰撞体(刚度1.0) ---")
	_ok("S4 物体穿透", worst_obj <= 0.0, "最大穿透=" + _fmt(worst_obj))
	_ok("S4 自碰瞬态(甩动中,≤30mm)", worst_self <= 0.03, "峰值=" + _fmt(worst_self))
	_ok("S4 自碰自愈(回稳后)", settle_self <= 0.0, "残余=" + _fmt(settle_self))
	_ok("S4 布间瞬态(甩动中,≤30mm)", worst_inter <= 0.03, "峰值=" + _fmt(worst_inter))
	_ok("S4 布间自愈(回稳后)", settle_inter <= 0.0, "残余=" + _fmt(settle_inter))
	_ok("S4 边界", _bounds_ok(), "NaN/地面")
