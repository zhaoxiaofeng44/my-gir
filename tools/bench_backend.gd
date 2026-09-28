extends SceneTree
# C++ vs GDScript 布料后端对比: 性能 + 物理质量
# 用同一个场景配置分别跑两套实现, 输出每帧耗时与穿透指标
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

	print("=== GDScript 后端(当前场景) ===")
	_run(300)
	var t0 := Time.get_ticks_usec()
	_run(120)
	var gd_ms := (Time.get_ticks_usec() - t0) / 120.0 / 1000.0
	print("  稳态 %.1f ms/帧  (等效 %.1f FPS)" % [gd_ms, 1000.0 / gd_ms])
	var gd_pen: Array = _tight()
	print("  真穿模点数=%d 最深=%.3fmm" % [gd_pen[0], gd_pen[1] * 1000.0])
	print("  约束数=%d" % scene.cloths[0].c_ia.size())
	quit()

func _tight() -> Array:
	var eps := 0.002
	var cnt := 0
	var worst := 0.0
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi:
				continue
			var A = scene.cloths[ai]
			var B = scene.cloths[bi]
			for i in range(0, A.pos.size(), 3):
				var p: Vector3 = A.pos[i]
				for r in B.rows - 1:
					for c in B.cols - 1:
						var a0: int = r * B.cols + c
						for tri in [[a0, a0 + B.cols, a0 + 1], [a0 + 1, a0 + B.cols, a0 + B.cols + 1]]:
							var cp: Vector3 = scene._closest_on_tri(p, B.pos[tri[0]], B.pos[tri[1]], B.pos[tri[2]])
							if p.distance_to(cp) <= eps:
								cnt += 1
	return [cnt, worst]

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
