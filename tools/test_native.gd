extends SceneTree
# C++ 后端独立验证: 建布 -> 积分 -> 求解 -> 自碰 -> 物体 -> 恢复, 测性能与稳定性
var cloth: XPBDClothNative
var col: XPBDCollider
var blob_root: Node3D

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	# 复现场景同样的形态: 24x24 落布 + 岩石碰撞体
	col = XPBDCollider.new()
	blob_root = Node3D.new()
	get_root().add_child(blob_root)
	col.root = blob_root
	var defs := [
		[Vector3(0, 0, 0), 0.42], [Vector3(0.26, 0.12, 0.08), 0.30],
		[Vector3(-0.24, 0.16, -0.06), 0.28], [Vector3(0.04, -0.22, 0.14), 0.30],
		[Vector3(0.08, 0.06, -0.28), 0.27], [Vector3(-0.1, -0.08, 0.26), 0.26]]
	for d in defs:
		col.add_sphere(d[0], d[1])
	blob_root.position = Vector3(0, 0.2, 0)
	blob_root.force_update_transform()

	cloth = XPBDClothNative.new()
	get_root().add_child(cloth)
	cloth.cols = 24
	cloth.rows = 24
	cloth.cloth_size = Vector2(2.0, 2.0)
	cloth.spawn_origin = Vector3(0, 1.8, 0)
	cloth.ground_y = -1.6
	cloth.gravity = Vector3(0, -9.8, 0)
	cloth.build()
	cloth.set_collider_shapes(col.pt, col.poff, col.pa, col.pb, col.pr, col.phalf, blob_root.global_transform)

	print("构建: 顶点=%d 约束=%d 三角形=%d tri_r=%.4f" % [
		cloth.get_positions().size(), cloth.get_constraint_count(),
		cloth.get_triangle_count(), cloth.get_tri_r()])

	# 模拟 10 秒(600 帧 x 4 子步)
	var t0 := Time.get_ticks_usec()
	for f in 600:
		_sim_frame(1.0 / 60.0)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 600.0
	print("10秒仿真: %.2f ms/帧 (等效 %.1f FPS)" % [ms, 1000.0 / ms])
	print("状态: 自碰欠量=%.4fmm 物体穿透点=%d 布间欠量=%.4fmm 撕裂=%d 休眠=%s" % [
		cloth.self_deficit() * 1000.0, cloth.penetration_count(),
		cloth.cross_deficit_all() * 1000.0, cloth.get_torn_count(), str(cloth.get_asleep())])
	# 速度收敛
	var mx := 0.0
	var vs: PackedVector3Array = cloth.get_velocities()
	for i in vs.size():
		mx = maxf(mx, vs[i].length())
	print("最大速度=%.2f m/s" % mx)
	quit()

func _sim_frame(delta: float) -> void:
	var h := delta / 4.0
	for s in 4:
		var last := s == 3
		cloth.sub_begin()
		cloth.sub_integrate(h, Vector3.ZERO)
		cloth.sub_solve(h)
		if last:
			cloth.begin_contacts()
			cloth.self_collision()
		cloth.object_collision()
		if last:
			cloth.final_push_out()
			cloth.cross_contact_all(true)
			cloth.self_collision()
			cloth.final_push_out()
		cloth.sub_recover(h)
