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
	_run(300)
	var cl: XPBDCloth = scene.cloths[0]
	print("布料: %dx%d=%d顶点  约束=%d" % [cl.cols, cl.rows, cl.pos.size(), cl.c_ia.size()])
	# 统计自碰: 每次 self_collision 里各轮扫掠的实际推挤次数与退出轮次
	var stats := _count_self_sweeps(cl)
	print("自碰单次调用: 扫描轮次=%d 各轮推挤数=%s" % [stats[0], str(stats[1])])
	# 约束类型分布
	var t := [0, 0, 0]
	for k in cl.c_type.size():
		t[cl.c_type[k]] += 1
	print("约束分布: 结构=%d 剪切=%d 弯曲=%d" % [t[0], t[1], t[2]])
	quit()

func _count_self_sweeps(cl: XPBDCloth) -> Array:
	# 复制 self_collision 的循环结构做计数(不改状态): 统计每轮实际参与的距离测试对数
	var n: int = cl.pos.size()
	var min_d: float = cl.cloth_thickness * 0.5 + cl.tri_r
	var min_d2: float = min_d * min_d
	var inv: float = 1.0 / min_d
	var pushes: Array = []
	for sweep in 6:
		var hsh := {}
		for i in n:
			var p: Vector3 = cl.pos[i]
			var key := (int(floor(p.x * inv)) + 512) << 20 | (int(floor(p.y * inv)) + 512) << 10 | (int(floor(p.z * inv)) + 512)
			if hsh.has(key):
				hsh[key].append(i)
			else:
				hsh[key] = [i]
		var pc := 0
		var tested := 0
		for i in n:
			var p: Vector3 = cl.pos[i]
			var cx := int(floor(p.x * inv)) + 512
			var cy := int(floor(p.y * inv)) + 512
			var cz := int(floor(p.z * inv)) + 512
			var ri: int = i / cl.cols
			var ci: int = i % cl.cols
			for off in XPBDCloth._OFFS:
				var key: int = (cx + off.x) << 20 | (cy + off.y) << 10 | (cz + off.z)
				if not hsh.has(key):
					continue
				for jj in hsh[key]:
					var j: int = jj
					if j <= i:
						continue
					var dr: int = absi(j / cl.cols - ri)
					var dc: int = absi(j % cl.cols - ci)
					if dr <= 2 and dc <= 2:
						continue
					tested += 1
					if p.distance_squared_to(cl.pos[j]) < min_d2:
						pc += 1
		pushes.append(pc)
		if pc == 0:
			break
	return [pushes.size(), pushes]

func _run(steps: int) -> void:
	for _s in steps:
		scene._run_sim(1.0 / 60.0)
