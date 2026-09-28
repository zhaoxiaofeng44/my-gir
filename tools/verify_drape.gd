extends SceneTree
# 最终质量验证: 布料落定后的形态(是否自然垂坠/不成团/无极限拉伸/透没透彻) + 静止收敛
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
	_run(600)   # 10秒: 充分落定
	var cl: XPBDCloth = scene.cloths[0]
	var lo := Vector3(9, 9, 9)
	var hi := Vector3(-9, -9, -9)
	for i in cl.pos.size():
		lo = lo.min(cl.pos[i])
		hi = hi.max(cl.pos[i])
	var mx := 0.0
	var fast := 0
	for c in scene.cloths:
		for i in c.vel.size():
			mx = maxf(mx, c.vel[i].length())
			if c.vel[i].length() > 0.3:
				fast += 1
	# 最大拉伸比
	var mx_stretch := 0.0
	var torn := 0
	for k in cl.c_ia.size():
		if cl.c_alive[k] == 0:
			torn += 1
			continue
		mx_stretch = maxf(mx_stretch, cl.pos[cl.c_ia[k]].distance_to(cl.pos[cl.c_ib[k]]) / cl.c_rest[k])
	print("落定形态: Y跨度=%.2f 尺寸=(%.2f, %.2f, %.2f)" % [hi.y - lo.y, hi.x - lo.x, hi.y - lo.y, hi.z - lo.z])
	print("最大拉伸比=%.3f (上限%.2f) 撕裂=%d 最大速度=%.2f 快速顶点=%d 休眠=%s" % [mx_stretch, cl.max_stretch, torn, mx, fast, str(cl._asleep)])
	quit()

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
