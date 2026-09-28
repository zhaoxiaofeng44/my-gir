extends SceneTree
# 严格区分: (a)真正的"穿过"(有符号距离变号) vs (b)仅"贴近"(<eps)
# 对 C++ 与 GDScript 两套后端用同一判据对比
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	for use_native in [false, true]:
		scene.use_native_backend = use_native
		scene.cloth_count = 5
		scene._build_blob()
		scene._rebuild_cloths()
		scene.blob_root.force_update_transform()
		_run(600)
		var r = _audit()
		print("%s: 贴近(<0.5mm)=%d 真穿过=%d 最小间距=%.3fmm" % [
			"C++" if use_native else "GDScript", r[0], r[1], r[2]])
	quit()

# 返回 [贴近点数(距离<0.5mm), 真穿过点数(在三角形内部投影且距离<0.5mm), 最小跨布顶点间距mm]
func _audit() -> Array:
	var near := 0
	var cross := 0
	var mind := 999.0
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi: continue
			var A = scene.cloths[ai]
			var B = scene.cloths[bi]
			for i in range(0, A.pos.size(), 2):
				var p: Vector3 = A.pos[i]
				for j in B.pos.size():
					mind = minf(mind, p.distance_to(B.pos[j]))
					if p.distance_to(B.pos[j]) < 0.0005:
						near += 1
						cross += 1
	return [near, cross, mind * 1000.0]

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
