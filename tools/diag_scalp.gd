extends SceneTree
# 头皮覆盖率: 发根在球面上的分布(前/后/左/右/顶)
var scene: Node

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	scene = (load("res://scenes/simple_hair_demo.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	scene.set_process(false)
	var hair: XPBDHairNative = scene.hair
	var vs: PackedVector3Array = hair.get_positions()
	var seg: int = hair.get_segment_count() + 1
	var guides: int = hair.get_guide_count()
	var hc: Vector3 = hair.get_head_center()
	# 统计发根方向分布(把局部坐标按象限归类)
	var quadrant := {"前(+Z)": 0, "后(-Z)": 0, "右(+X)": 0, "左(-X)": 0}
	var top := 0
	var low := 0
	for g in guides:
		var r: Vector3 = vs[g * seg] - hc
		var n: Vector3 = r.normalized()
		if absf(n.z) > absf(n.x):
			if n.z > 0:
				quadrant["前(+Z)"] += 1
			else:
				quadrant["后(-Z)"] += 1
		else:
			if n.x > 0:
				quadrant["右(+X)"] += 1
			else:
				quadrant["左(-X)"] += 1
		if n.y > 0.75:
			top += 1
		if n.y < 0.2:
			low += 1
	print("发根分布(共%d): %s | 近顶部=%d 近赤道=%d" % [guides, str(quadrant), top, low])
	# 渲染发丝分布
	var rr: int = hair.get_render_count()
	print("渲染发丝=%d 导向束=%d 比例=%d倍" % [rr, guides, rr / guides])
	quit()
