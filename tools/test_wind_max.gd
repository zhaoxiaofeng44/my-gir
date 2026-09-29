extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(300)
	# 极限: 最大阵风15 + 甩头 同时
	scene.hair.set_wind(Vector3(15, 2, 10))
	scene.shaking = true
	var head_in := 0
	var body_in := 0
	var worst_body := 0.0
	for f in 240:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			var skirt: XPBDClothNative = scene.skirt
			skirt.sub_begin()
			skirt.sub_integrate(sh, Vector3.ZERO)
			skirt.sub_solve(sh)
			skirt.begin_contacts()
			skirt.self_collision()
			skirt.final_push_out()
			skirt.sub_recover(sh)
		if f % 2 == 0:
			var vs: PackedVector3Array = scene.hair.get_positions()
			var hc: Vector3 = scene.hair.get_head_center()
			var ba: Vector3 = Vector3(0, 1.42, 0)
			var bb: Vector3 = Vector3(0, 0.55, 0)
			var br: float = 0.178
			for i in vs.size():
				var p: Vector3 = vs[i]
				if p.distance_to(hc) < 0.118:
					head_in += 1
				var raw: float = (p - ba).dot(bb - ba) / (bb - ba).length_squared()
				var t: float = minf(maxf(raw, 0.0), 1.0)
				var near: Vector3 = ba + (bb - ba) * t
				var d: float = p.distance_to(near)
				if t > 0.001 and t < 0.999 and d < br - 0.005:
					body_in += 1
					worst_body = maxf(worst_body, br - 0.005 - d)
	scene.shaking = false
	scene.hair.set_wind(Vector3())
	_run(180)
	var hvs: PackedVector3Array = scene.hair.get_positions()
	var inside := 0
	for i in hvs.size():
		if hvs[i].distance_to(Vector3(0, 1.55, 0)) < 0.118:
			inside += 1
	print("阵风15+甩头: 头内=%d 身体内=%d(最深%.2fmm)" % [head_in, body_in, worst_body * 1000.0])
	print("回稳后: 头内粒子=%d" % inside)
	print("物体穿透点=", scene.pen_count)
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
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
