extends SceneTree
# 场景级 A/B: C++ 后端 vs GDScript 后端
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)

	for use_native in [false, true]:
		scene.use_native_backend = use_native
		scene.cloth_count = 2
		scene._build_blob()
		scene._rebuild_cloths()
		scene.blob_root.force_update_transform()
		_run(300)
		var t0 := Time.get_ticks_usec()
		_run(180)
		var ms := (Time.get_ticks_usec() - t0) / 180.0 / 1000.0
		var name := "C++" if use_native else "GDScript"
		var impl = scene.cloths[0]
		print("%s: %.2f ms/帧 (%.1f FPS) 布料数=%d 顶点=%d 约束=%d" % [
			name, ms, 1000.0 / ms, scene.cloths.size(),
			impl.pos.size(), impl.c_ia.size()])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
