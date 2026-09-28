extends SceneTree
# 真实场景静置后的 抖动/性能/穿透 测量(全速 headless)
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
	_run(900)   # 15秒: 充分落定
	var out: Array = []
	for cl in scene.cloths:
		var hi := 0
		var mx := 0.0
		for i in cl.vel.size():
			var l: float = cl.vel[i].length()
			mx = maxf(mx, l)
			if l > 0.3:
				hi += 1
		out.append({"name": cl.name, "asleep": cl._asleep, "maxvel": snappedf(mx, 0.01), "fast": hi, "verts": cl.pos.size()})
	var t0 := Time.get_ticks_usec()
	_run(60)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 60.0
	print("静置15秒后: %s" % str(out))
	print("稳态性能: %.1fms/帧 (60fps预算16.7ms)" % ms)
	# 穿透
	print("物体穿透点=%d 布间穿模=%s" % [scene.pen_count, str(scene.face_cross_count())])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
