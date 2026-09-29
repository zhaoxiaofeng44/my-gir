extends SceneTree
var root_node: Node3D

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	root_node = Node3D.new()
	get_root().add_child(root_node)
	var script: GDScript = load("res://hs2/hs2_character.gd")
	root_node.set_script(script)
	for i in 30:
		await process_frame
	# 找所有 MeshInstance3D 并算全局 AABB
	var mmi: Array = []
	var stack: Array = [root_node]
	while stack.size() > 0:
		var n = stack.pop_back()
		for c in n.get_children():
			stack.append(c)
		if n is MeshInstance3D:
			mmi.append(n)
	print("MeshInstance3D 数量=", mmi.size())
	var lo := Vector3(9e9, 9e9, 9e9)
	var hi := Vector3(-9e9, -9e9, -9e9)
	var any_visible := 0
	for m in mmi:
		if m.visible:
			any_visible += 1
		var aabb: AABB = m.global_transform * m.get_aabb()
		lo = lo.min(aabb.position)
		hi = hi.max(aabb.position + aabb.size)
	print("可见网格=", any_visible, "/", mmi.size())
	print("全局 AABB: min=", lo, " max=", hi)
	var sk: Skeleton3D = null
	for m in mmi:
		var p = m
		while p:
			if p is Skeleton3D:
				sk = p
				break
			p = p.get_parent()
	if sk:
		var hb = sk.find_bone("cf_J_Head_s")
		if hb < 0: hb = sk.find_bone("cf_J_Head")
		if hb >= 0:
			print("头骨全局位置=", sk.get_bone_global_pose(hb).origin)
		var hp = sk.find_bone("cf_J_Hips")
		if hp >= 0:
			print("髋骨全局位置=", sk.get_bone_global_pose(hp).origin)
	else:
		print("NO_SK")
	quit()
