extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_run(600)
	var hair: XPBDHairNative = scene.hair
	var vs: PackedVector3Array = hair.get_positions()
	# 身体胶囊: 轴 (0,1.42,0)-(0,0.55,0), 半径 0.17
	var ba: Vector3 = Vector3(0, 1.42, 0)
	var bb: Vector3 = Vector3(0, 0.55, 0)
	var br: float = 0.17 + 0.008   # + 发丝碰撞半径
	var inside := 0
	var worst := 0.0
	for i in vs.size():
		var p: Vector3 = vs[i]
		var raw: float = (p - ba).dot(bb - ba) / (bb - ba).length_squared()
		var t: float = minf(maxf(raw, 0.0), 1.0)
		var near: Vector3 = ba + (bb - ba) * t
		var d: float = p.distance_to(near)
		if t > 0.001 and t < 0.999 and d < br:
			inside += 1
			worst = maxf(worst, br - d)
	print("身体胶囊内头发粒子=%d (应0) 最深=%.2fmm" % [inside, worst * 1000.0])
	# 头部
	var hc := Vector3(0, 1.55, 0)
	var hin := 0
	for i in vs.size():
		if vs[i].distance_to(hc) < 0.118:
			hin += 1
	print("头内粒子=%d (应0)" % hin)
	# 性能
	var t0 := Time.get_ticks_usec()
	_run(120)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 120.0
	print("性能=%.2f ms/帧" % ms)
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene.hair.step(1.0 / 60.0, 3)
		var sh: float = 1.0 / 120.0
		for ss in 2:
			scene.skirt.sub_begin()
			scene.skirt.sub_integrate(sh, Vector3.ZERO)
			scene.skirt.sub_solve(sh)
			scene.skirt.begin_contacts()
			scene.skirt.self_collision()
			scene.skirt.final_push_out()
			scene.skirt.sub_recover(sh)
		scene.hair.update_mesh()
		scene.skirt.update_mesh()
