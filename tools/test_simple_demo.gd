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
	var hvs: PackedVector3Array = hair.get_positions()
	var hc: Vector3 = Vector3(0, 1.55, 0)
	var h_mx := 0.0
	var inside := 0
	for i in hvs.size():
		h_mx = maxf(h_mx, hvs[i].length())
		if hvs[i].distance_to(hc) < 0.118:
			inside += 1
	var t0 := Time.get_ticks_usec()
	_run(60)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 60.0
	print("头发: 最大速度=%.2f 头内粒子=%d" % [h_mx, inside])
	print("性能: %.1f ms/帧" % ms)
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		# 场景没有 _run_sim, 手动驱动各部分
		scene.hair.step(1.0 / 60.0, 3)
		var sh := (1.0 / 60.0) / 2.0
		var skirt: XPBDClothNative = scene.skirt
		for ss in 2:
			skirt.sub_begin()
			skirt.sub_integrate(sh, Vector3.ZERO)
			skirt.sub_solve(sh)
			skirt.begin_contacts()
			skirt.self_collision()
			skirt.final_push_out()
			skirt.sub_recover(sh)
		scene.hair.update_mesh()
		scene.skirt.update_mesh()
		scene.hair_mesh.mesh = scene.hair.get_mesh()
		scene.skirt_mesh.mesh = scene.skirt.get_mesh()
		scene._frame_count += 1
