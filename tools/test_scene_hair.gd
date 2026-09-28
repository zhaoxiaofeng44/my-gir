extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	var t0 := Time.get_ticks_usec()
	_run(240)   # 4秒
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 240.0
	var out = {}
	out["hair_active"] = scene.hair_native != null
	out["guides"] = 0
	out["render"] = 0
	if scene.hair_native != null:
		out["guides"] = scene.hair_native.get_guide_count()
		out["render"] = scene.hair_native.get_render_count()
		out["hair_verts"] = scene.hair_native.get_positions().size()
	# 头发粒子不应穿过头部
	var inside := 0
	if scene.hair_native != null:
		var vs: PackedVector3Array = scene.hair_native.get_positions()
		var hc := Vector3(0, 0.55, 0)
		for i in vs.size():
			if vs[i].distance_to(hc) < 0.115:
				inside += 1
	out["head_inside"] = inside
	out["frame_ms"] = snappedf(ms, 0.1)
	print("SCENE ", str(out))
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
