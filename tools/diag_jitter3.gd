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
	# 找最大速度顶点, 逐子步追踪它的速度从哪来
	var mi := 0
	var mx := 0.0
	for i in cl.vel.size():
		if cl.vel[i].length() > mx:
			mx = cl.vel[i].length()
			mi = i
	print("最大速度顶点 %d: v=%s pos=%s pinned=%d grab=%d" % [mi, str(cl.vel[mi]), str(cl.pos[mi]), cl.pinned[mi], cl.grab_id])
	# 追踪一帧内的位置/速度变化
	for sub in 4:
		var h := 1.0 / 240.0
		cl.sub_begin()
		cl.sub_integrate(h, Vector3.ZERO)
		var after_int := cl.vel[mi].length()
		cl.sub_solve(h)
		var after_solve := cl.vel[mi].length()
		cl.begin_contacts()
		cl.self_collision()
		var after_self := cl.vel[mi].length()
		var pushed = cl._pushed[mi]
		var pushed_soft = cl._pushed_soft[mi]
		cl.sub_recover(h)
		print("  子步%d: 积分后|v|=%.3f 求解后=%.3f 自碰后=%.3f pushed=%d soft=%d 最终v=%s" % [sub, after_int, after_solve, after_self, pushed, pushed_soft, str(cl.vel[mi])])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
