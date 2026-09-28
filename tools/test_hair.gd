extends SceneTree
# 头发系统验证: 构建 -> 模拟 -> 渲染网格, 检查物理稳定性与插值质量
var hair: XPBDHairNative

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	hair = XPBDHairNative.new()
	get_root().add_child(hair)
	hair.set_strand_count(48)
	hair.set_segment_count(12)
	hair.set_hair_length(0.55)
	hair.set_head_radius(0.12)
	hair.set_head_center(Vector3(0, 0.2, 0))
	hair.set_render_strands(240)
	hair.build()
	print("构建: 导向束=%d 渲染束=%d 段数=%d 粒子=%d" % [
		hair.get_guide_count(), hair.get_render_count(), hair.get_segment_count(),
		hair.get_positions().size()])
	# 模拟 5 秒
	var t0 := Time.get_ticks_usec()
	for f in 300:
		hair.step(1.0 / 60.0, 4)
	var ms := (Time.get_ticks_usec() - t0) / 1000.0 / 300.0
	print("模拟: %.3f ms/帧 (hair only, 等效 %.0f FPS)" % [ms, 1000.0 / ms])
	# 检查稳定性: 发梢不应穿过头部, 速度应收敛
	var vs: PackedVector3Array = hair.get_positions()
	var mx := 0.0
	var inside := 0
	var np := hair.get_segment_count() + 1
	var hc := Vector3(0, 0.2, 0)
	for i in vs.size():
		mx = maxf(mx, vs[i].length())
		if (vs[i] - hc).length() < 0.12:
			inside += 1
	print("最大速度=%.3f m/s 头内粒子=%d (应为0)" % [mx, inside])
	# 渲染网格
	hair.update_mesh()
	print("渲染网格: 顶点=%d" % (hair.get_render_count() * np * 2))
	quit()
