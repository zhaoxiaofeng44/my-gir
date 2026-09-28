extends SceneTree
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	for force in [1, 2, 4, 6]:
		scene.cloth_count = 5
		scene._build_blob()
		scene._rebuild_cloths()
		scene.blob_root.force_update_transform()
		# 强制自碰轮数
		for cl in scene.cloths:
			cl.set_meta("sweeps", force)
		var t0 := Time.get_ticks_usec()
		_run(600)
		var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 600.0
		var res: Array = _tight()
		print("轮数=%d: %.1fms/帧  真穿模=%d点 最深=%.3fmm" % [force, ms, res[0], res[1] * 1000.0])
	quit()

func _tight() -> Array:
	var eps := 0.002
	var cnt := 0
	var worst := 0.0
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi:
				continue
			var A: XPBDCloth = scene.cloths[ai]
			var B: XPBDCloth = scene.cloths[bi]
			for i in range(0, A.pos.size(), 3):
				var p: Vector3 = A.pos[i]
				for r in B.rows - 1:
					for c in B.cols - 1:
						var a0: int = r * B.cols + c
						for tri in [[a0, a0 + B.cols, a0 + 1], [a0 + 1, a0 + B.cols, a0 + B.cols + 1]]:
							var cp: Vector3 = scene._closest_on_tri(p, B.pos[tri[0]], B.pos[tri[1]], B.pos[tri[2]])
							if p.distance_to(cp) > eps:
								continue
							var nrm: Vector3 = (B.pos[tri[1]] - B.pos[tri[0]]).cross(B.pos[tri[2]] - B.pos[tri[0]])
							if nrm.length() < 1e-9:
								continue
							cnt += 1
	return [cnt, worst]

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
