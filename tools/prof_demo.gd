extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	for f in 300:
		_sim(1.0 / 60.0)
	# 分阶段计时(120帧平均)
	var t_hair := 0
	var t_int := 0
	var t_solve := 0
	var t_self := 0
	var t_other := 0
	for f in 120:
		var dt := 1.0 / 60.0
		var t0 := Time.get_ticks_usec()
		scene.hair.step(dt, 3)
		var t1 := Time.get_ticks_usec()
		var sh: float = dt / 2.0
		var skirt: XPBDClothNative = scene.skirt
		for ss in 2:
			skirt.sub_begin()
			var a := Time.get_ticks_usec()
			skirt.sub_integrate(sh, Vector3.ZERO)
			var b := Time.get_ticks_usec()
			skirt.sub_solve(sh)
			var c := Time.get_ticks_usec()
			skirt.begin_contacts()
			skirt.self_collision()
			var d := Time.get_ticks_usec()
			skirt.final_push_out()
			skirt.collide_body_capsule()
			skirt.sub_recover(sh)
			var e := Time.get_ticks_usec()
			t_int += b - a
			t_solve += c - b
			t_self += d - c
			t_other += e - d
		t_hair += t1 - t0
	print("分阶段(ms/帧): 头发=%.2f 裙布积分=%.2f 裙布约束=%.2f 裙布自碰=%.2f 裙布其他=%.2f" % [
		t_hair / 120.0 / 1000.0, t_int / 120.0 / 1000.0, t_solve / 120.0 / 1000.0,
		t_self / 120.0 / 1000.0, t_other / 120.0 / 1000.0])
	var total: float = (t_hair + t_int + t_solve + t_self + t_other) / 120.0 / 1000.0
	print("物理总计=%.2f ms/帧 (等效 %.0f FPS)" % [total, 1000.0 / total])
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
		skirt.collide_body_capsule()
		skirt.sub_recover(sh)
	scene.hair.update_mesh()
	scene.skirt.update_mesh()
