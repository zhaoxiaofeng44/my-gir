extends SceneTree
# 紧判据(容差2mm)下的多层重叠全场景验证: 静置/拖拽穿层/五层强制叠压
var scene: Node
var fail := 0

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	_make_layers()
	_run(600)
	_tight("静置叠放")
	scene.collider_mode = 2
	scene._build_blob()
	scene.blob_root.force_update_transform()
	_run(60)
	scene.drag_collider = true
	scene.drag_speed = 4.0
	for target: Vector3 in [Vector3(-1.6, 0.9, 0.3), Vector3(1.6, 0.7, -0.3), Vector3(0, 1.2, 0)]:
		scene.drag_pos = target
		_run(100)
	scene.drag_collider = false
	_run(100)
	_tight("拖拽穿层后")
	_make_layers()
	_run(400)
	for s in 90:
		for cl in scene.cloths:
			for i in cl.pos.size():
				cl.vel[i] = Vector3(0, -3.0, 0)
		_run(1)
	_run(150)
	_tight("五层强制叠压后")
	print("========== 紧判据结论: %s ==========" % ("全部通过" if fail == 0 else "%d 项失败" % fail))
	quit()

func _make_layers() -> void:
	scene.cloth_count = 5
	scene._build_blob()
	scene._rebuild_cloths()
	scene.blob_root.force_update_transform()
	var offs: Array = [Vector3(0, 1.6, 0), Vector3(0.12, 2.2, -0.08), Vector3(-0.1, 2.8, 0.1), Vector3(0.08, 3.4, -0.05), Vector3(-0.06, 4.0, 0.08)]
	for k in 5:
		var cl: XPBDCloth = scene.cloths[k]
		cl.vertical = false
		cl.cols = 20
		cl.rows = 20
		cl.spawn_origin = offs[k]
		cl.build()

func _tight(tag: String) -> void:
	var eps := 0.002
	var cnt := 0
	var worst := 0.0
	for ai in scene.cloths.size():
		for bi in scene.cloths.size():
			if ai == bi:
				continue
			var A: XPBDCloth = scene.cloths[ai]
			var B: XPBDCloth = scene.cloths[bi]
			for i in range(0, A.pos.size(), 2):
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
							nrm = nrm.normalized()
							var sd: float = (p - cp).dot(nrm)
							var d1: float = cp.distance_to(XPBDCollider.closest_on_seg(B.pos[tri[0]], B.pos[tri[1]], cp))
							var d2: float = cp.distance_to(XPBDCollider.closest_on_seg(B.pos[tri[1]], B.pos[tri[2]], cp))
							var d3: float = cp.distance_to(XPBDCollider.closest_on_seg(B.pos[tri[2]], B.pos[tri[0]], cp))
							if d1 > 1e-6 and d2 > 1e-6 and d3 > 1e-6:
								cnt += 1
								worst = maxf(worst, absf(sd))
	if cnt > 0:
		fail += 1
	print("[%s] 真穿模点=%d 最深=%.3fmm" % [tag, cnt, worst * 1000.0])

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
