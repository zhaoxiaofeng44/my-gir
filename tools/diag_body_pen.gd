extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(300)
	scene.wind_level = 3
	scene.shaking = true
	for f in 240:
		var d: Vector3 = Vector3(sin(f * 0.08) * 0.02, 0, cos(f * 0.06) * 0.015)
		scene._apply_model_delta(d)
		_sim(1.0 / 60.0)
	scene.shaking = false
	# 详细诊断
	var hair: XPBDHairNative = scene.hair
	var vs: PackedVector3Array = hair.get_positions()
	var hc: Vector3 = hair.get_head_center()
	var ba: Vector3 = hair.get_body_a()
	var bb: Vector3 = hair.get_body_b()
	print("C++ 头心=", hc, " 胶囊A=", ba, " B=", bb, " body_on=", hair.get_body_on())
	print("场景 head.pos=", scene.head.position, " body.pos=", scene.body.position)
	var bad := 0
	for i in vs.size():
		var p: Vector3 = vs[i]
		var raw: float = (p - ba).dot(bb - ba) / (bb - ba).length_squared()
		var t: float = minf(maxf(raw, 0.0), 1.0)
		if t > 0.001 and t < 0.999:
			var d: float = p.distance_to(ba + (bb - ba) * t)
			if d < 0.168:
				bad += 1
				if bad <= 5:
					print("  穿入顶点#", i, " pos=", p, " t=", snappedf(t, 0.01), " d=", snappedf(d * 1000.0, 0.1), "mm")
	print("总计=", bad)
	quit()

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
