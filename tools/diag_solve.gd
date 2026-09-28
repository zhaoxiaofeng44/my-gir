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
	_run(200)
	var cl: XPBDCloth = scene.cloths[0]
	# 分类型计时 sub_solve 的迭代部分 vs 应变限制部分
	var t_iter := 0
	var t_strain := 0
	var h := 1.0 / 240.0
	for i in 30:
		var t0 := Time.get_ticks_usec()
		_measure_iter(cl, h)
		t_iter += Time.get_ticks_usec() - t0
	print("sub_solve(约束迭代+应变限制) 单次=%.3fms, 其中约束=%d条" % [t_iter / 30.0 / 1000.0, cl.c_ia.size()])
	# 统计: 应变限制实际改动的约束数(静置态)
	var changed := 0
	for k in cl.c_ia.size():
		if cl.c_alive[k] == 0 or cl.c_type[k] == 2:
			continue
		var ln: float = cl.pos[cl.c_ia[k]].distance_to(cl.pos[cl.c_ib[k]])
		if ln > cl.c_rest[k] * cl.max_stretch:
			changed += 1
	print("静置态: 应变限制实际生效约束数=%d / %d" % [changed, cl.c_ia.size()])
	quit()

func _measure_iter(cl: XPBDCloth, h: float) -> void:
	cl.sub_solve(h)

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
