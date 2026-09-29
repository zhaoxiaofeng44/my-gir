extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	# 稳定
	_run(300)
	print("== 测试1: 拖拽模型(快速左右移动1.5m) + 强风 ==")
	scene.wind_level = 2
	# 模拟拖拽: 60 帧内把模型从 0 拖到 (1.5, 0, 0), 再拖回
	var head0: Vector3 = scene.head.position
	for half in 2:
		var sign_f: float = 1.0 if half == 0 else -1.0
		for f in 60:
			var d: Vector3 = Vector3(sign_f * 1.5 / 60.0, 0, 0)
			scene._apply_model_delta(d)
			_sim(1.0 / 60.0)
	# 拖拽中审计
	var r: Array = _audit()
	print("拖拽后: 头内=%d 身体内=%d(最深%.2fmm)" % [r[0], r[1], r[2] * 1000.0])
	print("== 测试2: 拖拽中甩头 + 阵风15 ==")
	scene.wind_level = 3
	scene.shaking = true
	var h2 := 0
	var b2 := 0
	var wb := 0.0
	for f in 240:
		# 模型来回晃
		var d: Vector3 = Vector3(sin(f * 0.08) * 0.02, 0, cos(f * 0.06) * 0.015)
		scene._apply_model_delta(d)
		_sim(1.0 / 60.0)
		if f % 2 == 0:
			var a2: Array = _audit()
			h2 += a2[0]
			b2 += a2[1]
			wb = maxf(wb, a2[2])
	scene.shaking = false
	scene.wind_level = 0
	print("晃动240帧(每2帧审计): 头内=%d 身体内=%d 最深=%.2fmm" % [h2, b2, wb * 1000.0])
	_run(180)
	var r3: Array = _audit()
	print("回稳后: 头内=%d 身体内=%d 最深=%.2fmm" % [r3[0], r3[1], r3[2] * 1000.0])
	# 性能
	var t0 := Time.get_ticks_usec()
	_sim(120)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 120.0
	print("性能=%.2f ms/帧(头发3子步+裙布2子步)" % ms)
	quit()

func _audit() -> Array:
	var hair: XPBDHairNative = scene.hair
	var vs: PackedVector3Array = hair.get_positions()
	var hc: Vector3 = hair.get_head_center()
	var ba: Vector3 = Vector3(scene.head.position.x, 1.42, scene.head.position.z)
	var bb: Vector3 = Vector3(scene.body.position.x, 0.55, scene.body.position.z)
	var br: float = 0.173
	var hi := 0
	var bi := 0
	var worst := 0.0
	for i in vs.size():
		var p: Vector3 = vs[i]
		if p.distance_to(hc) < 0.118:
			hi += 1
		var raw: float = (p - ba).dot(bb - ba) / (bb - ba).length_squared()
		var t: float = minf(maxf(raw, 0.0), 1.0)
		if t > 0.001 and t < 0.999:
			var d: float = p.distance_to(ba + (bb - ba) * t)
			if d < br - 0.005:
				bi += 1
				worst = maxf(worst, br - 0.005 - d)
	return [hi, bi, worst]

func _sim(dt: float) -> void:
	scene.hair.step(dt, 3)
	var sh: float = dt / 2.0
	for ss in 2:
		var skirt: XPBDClothNative = scene.skirt
		skirt.sub_begin()
		skirt.sub_integrate(sh, Vector3.ZERO)
		skirt.sub_solve(sh)
		skirt.begin_contacts()
		skirt.self_collision()
		skirt.final_push_out()
		skirt.sub_recover(sh)
	scene.hair.update_mesh()
	scene.skirt.update_mesh()

func _run(steps: int) -> void:
	for _s in steps:
		_sim(1.0 / 60.0)
