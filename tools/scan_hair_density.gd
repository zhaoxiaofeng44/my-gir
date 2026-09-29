extends SceneTree
# 头发密度扫描: 不同渲染发丝数下的 性能 / 穿模 / 头皮间距
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	var hair: XPBDHairNative = scene.hair
	for n in [300, 800, 1500, 2500, 4000]:
		hair.set_render_strands(n)
		hair.build()
		# 稳定
		for f in 180:
			_sim(1.0 / 60.0)
		# 计时
		var t0 := Time.get_ticks_usec()
		for f in 120:
			_sim(1.0 / 60.0)
		var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 120.0
		# 穿模审计
		var vs: PackedVector3Array = hair.get_positions()
		var hc: Vector3 = hair.get_head_center()
		var inside := 0
		for i in vs.size():
			if vs[i].distance_to(hc) < 0.118:
				inside += 1
		# 网格顶点数
		var verts: int = hair.get_render_count() * (hair.get_segment_count() + 1) * 2
		print("渲染=%4d束: 物理+渲染=%.2fms/帧 头内=%d 网格顶点=%d" % [
			hair.get_render_count(), ms, inside, verts])
	quit()

func _sim(dt: float) -> void:
	scene.hair.step(dt, 3)
	var sh: float = dt / 2.0
	for ss in 2:
		var skirt: XPBDClothNative = scene.skirt
		skirt.sub_begin()
		skirt.sub_integrate(sh, Vector3.ZERO)
		skirt.sub_solve(sh)
		skirt.begin_contacts()
		skirt.self_collision()
		skirt.final_push_out()
		skirt.collide_body_capsule()
		skirt.sub_recover(sh)
	scene.hair.update_mesh()
	scene.skirt.update_mesh()
