extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	scene._build_blob()
	scene._rebuild_cloths()
	scene.blob_root.force_update_transform()
	_run(900)
	var cl: XPBDCloth = scene.cloths[0]
	print("tri_r=%.4f 自碰min_d=%.4f" % [cl.tri_r, cl.cloth_thickness * 0.5 + cl.tri_r])
	# 单次 self_collision 的耗时(稳态)
	var t_total := 0
	for i in 10:
		var t0 := Time.get_ticks_usec()
		cl.self_collision()
		t_total += Time.get_ticks_usec() - t0
	print("self_collision 单次=%.2fms (Sheet 576顶点)" % [t_total / 10.0 / 1000.0])
	# 各阶段的耗时占比(单帧)
	var t_solve := 0
	var t_self := 0
	var t_cross := 0
	var t_obj := 0
	for s in 10:
		var t0 := Time.get_ticks_usec()
		for c in scene.cloths:
			c.sub_begin()
			c.sub_integrate(1.0 / 240.0, Vector3.ZERO)
			c.sub_solve(1.0 / 240.0)
		t_solve += Time.get_ticks_usec() - t0
		t0 = Time.get_ticks_usec()
		for c in scene.cloths:
			c.self_collision()
		t_self += Time.get_ticks_usec() - t0
		t0 = Time.get_ticks_usec()
		scene._cross_contact(true)
		t_cross += Time.get_ticks_usec() - t0
		t0 = Time.get_ticks_usec()
		for c in scene.cloths:
			c.object_collision()
			c.final_push_out()
		t_obj += Time.get_ticks_usec() - t0
	print("单帧: solve=%.1f self=%.1f cross=%.1f obj=%.1f ms" % [t_solve / 10.0 / 1000.0, t_self / 10.0 / 1000.0, t_cross / 10.0 / 1000.0, t_obj / 10.0 / 1000.0])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
