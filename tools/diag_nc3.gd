extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	scene.use_native_backend = true
	scene.cloth_count = 5
	scene._build_blob()
	scene._rebuild_cloths()
	scene.blob_root.force_update_transform()
	var impl = scene._native_impl
	print("注册布料数 = ", impl.registry_size())
	print("跨布诊断 [注册数,候选对,AABB相交对,阈值um] = ", impl.cross_debug())
	_run(600)
	print("600帧后 注册数 = ", impl.registry_size())
	print("600帧后 诊断 = ", impl.cross_debug())
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
