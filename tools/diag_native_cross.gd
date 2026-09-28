extends SceneTree
# 诊断 C++ 后端下的布间状态: cross 是否被调用? 实际欠量多少?
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
	_run(600)
	var impl = scene._native_impl
	print("native impl: ", impl != null)
	if impl:
		print("布间欠量(C++) = %.4fmm" % (impl.cross_deficit_all() * 1000.0))
		print("布间候选对数 = ", impl.cross_contact_all(false))
	# 顶点口径: 各布对最小顶点距离
	var md := 999.0
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi: continue
			var A = scene.cloths[ai]
			var B = scene.cloths[bi]
			for i in A.pos.size():
				var p = A.pos[i]
				for j in B.pos.size():
					md = minf(md, p.distance_to(B.pos[j]))
	print("全体最小跨布顶点距离 = %.2fmm (判据=%.2fmm)" % [md * 1000.0, impl.cross_deficit_all() * 0.0 + md * 1000.0])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
