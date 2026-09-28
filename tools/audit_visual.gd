extends SceneTree
# 密集网格级视觉穿透审计: 三角形边(5点)+面内重心(10点) vs 视觉(未膨胀)碰撞面
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
	for m in 3:
		_setup(m)
		_run(300)
		_audit(m)
	_setup(0)
	_run(200)
	scene.drag_collider = true
	scene.drag_speed = 6.0
	scene.drag_pos = Vector3(0, 2.0, 0)
	_run(60)
	scene.drag_pos = Vector3(0, -1.2, 0)
	_run(40)
	scene.drag_collider = false
	_run(90)
	_audit(99)
	print("========== 视觉审计结论: %d/%d PASS ==========" % [cases - fail, cases])
	quit()

func _setup(mode: int) -> void:
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
	scene.cloth_count = 2
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

func _depth_at(col: XPBDCollider, p_l: Vector3) -> float:
	var d := 0.0
	for k in col.count:
		var ty: int = col.pt[k]
		var dd := 0.0
		if ty == 0:
			dd = col.pr[k] - (p_l - col.poff[k]).length()
		elif ty == 1:
			dd = col.pr[k] - (p_l - XPBDCollider.closest_on_seg(col.pa[k], col.pb[k], p_l)).length()
		else:
			var l := p_l - col.poff[k]
			var h := col.phalf[k]
			if absf(l.x) < h.x and absf(l.y) < h.y and absf(l.z) < h.z:
				dd = minf(minf(h.x - absf(l.x), h.y - absf(l.y)), h.z - absf(l.z))
		if dd > d:
			d = dd
	return d

func _audit(m: int) -> void:
	var col: XPBDCollider = scene.collider
	var xfi: Transform3D = col.root.global_transform.affine_inverse()
	var worst := 0.0
	var cnt := 0
	var worst_p := Vector3.ZERO
	for cl in scene.cloths:
		var cols: int = cl.cols
		for r_i in cl.rows - 1:
			for c_i in cols - 1:
				var a0: int = r_i * cols + c_i
				var q: Array = [a0, a0 + cols, a0 + 1, a0 + cols + 1]
				var tris: Array = [[q[0], q[1], q[2]], [q[2], q[1], q[3]]]
				for tri in tris:
					var A: Vector3 = cl.pos[tri[0]]
					var B: Vector3 = cl.pos[tri[1]]
					var C: Vector3 = cl.pos[tri[2]]
					for su in 4:
						for sv in 4 - su:
							var w0 := float(su) / 3.0
							var w1 := float(sv) / 3.0
							var smp: Vector3 = A * w0 + B * w1 + C * (1.0 - w0 - w1)
							var d := _depth_at(col, xfi * smp)
							if d > worst:
								worst = d
								worst_p = smp
							if d > 0.0:
									cnt += 1
					for pair in [[A, B], [B, C], [C, A]]:
						for t in 5:
							var smp2: Vector3 = pair[0].lerp(pair[1], float(t) / 4.0)
							var d2 := _depth_at(col, xfi * smp2)
							if d2 > worst:
								worst = d2
								worst_p = smp2
							if d2 > 0.0:
									cnt += 1
	print("--- V%d 视觉穿透(网格级, 未膨胀面) ---" % m)
	_ok("V%d 视觉穿透" % m, worst <= 0.0005, "最深=%.2fmm 采样点数=%d 最深点=%s" % [worst * 1000.0, cnt, worst_p])
