extends SceneTree
# C++ 后端: 布间判据是否真的用了几何补偿? 打印实际阈值
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
	print("tri_r 各布料: ", [scene.cloths[0].tri_r, scene.cloths[1].tri_r])
	print("厚度=%.4f => 应有阈值 = 厚度*0.5 + maxR = %.4f" % [
		scene.cloths[0].cloth_thickness, scene.cloths[0].cloth_thickness * 0.5 + scene.cloths[0].tri_r])
	_run(600)
	var impl = scene._native_impl
	print("C++ 布间欠量 = %.4fmm" % (impl.cross_deficit_all() * 1000.0))
	# 手工按补偿阈值验证: 找出最小跨布顶点距离对应的两顶点
	var md := 999.0
	var pair := []
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi: continue
			var A = scene.cloths[ai]
			var B = scene.cloths[bi]
			for i in A.pos.size():
				var p = A.pos[i]
				for j in B.pos.size():
					var d = p.distance_to(B.pos[j])
					if d < md:
						md = d
						pair = [ai, i, bi, j]
	print("最小跨布顶点距离=%.3fmm 对=%s" % [md * 1000.0, str(pair)])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
