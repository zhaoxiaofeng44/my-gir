extends SceneTree
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
	_run(900)
	var cl: XPBDCloth = scene.cloths[0]
	var pad: float = cl.cloth_thickness * 0.5 + cl.collision_margin
	var gy: float = cl.ground_y + pad
	var on_ground := 0
	var near_rock := 0
	var free_jit := 0
	var mx_free := 0.0
	var free_pos := Vector3.ZERO
	for i in cl.pos.size():
		if cl.vel[i].length() <= 0.3:
			continue
		var p: Vector3 = cl.pos[i]
		if p.y < gy + 0.01:
			on_ground += 1
		else:
			# 距岩石表面
			var d: float = 9.0
			for k in scene.collider.count:
				var dd: float = (p - scene.collider.poff[k]).length() - scene.collider.pr[k]
				d = minf(d, dd)
			if d < 0.05:
				near_rock += 1
			else:
				free_jit += 1
				if cl.vel[i].length() > mx_free:
					mx_free = cl.vel[i].length()
					free_pos = p
	print("抖动顶点归属: 贴地=%d 近岩石=%d 悬空自由=%d (最大%.2f at %s)" % [on_ground, near_rock, free_jit, mx_free, str(free_pos)])
	print("地面线 y=%.3f, 最大速度顶点=%s" % [gy, str(cl.vel[0])])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
