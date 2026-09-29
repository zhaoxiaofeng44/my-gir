extends SceneTree
var root_node: Node3D

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	root_node = Node3D.new()
	get_root().add_child(root_node)
	var script: GDScript = load("res://hs2/hs2_character.gd")
	root_node.set_script(script)
	for i in 5:
		await process_frame
	var sk: Skeleton3D = null
	var stack: Array = [root_node]
	while stack.size() > 0:
		var n = stack.pop_back()
		for c in n.get_children():
			stack.append(c)
		if n is Skeleton3D:
			sk = n
			break
	if sk == null:
		print("NO_SKELETON")
	else:
		print("骨架骨骼数=", sk.get_bone_count())
		var hb = sk.find_bone("cf_J_Head_s")
		if hb < 0:
			hb = sk.find_bone("cf_J_Head")
		if hb >= 0:
			print("头骨 idx=", hb, " 全局位置=", sk.get_bone_global_pose(hb).origin)
		else:
			print("NO_HEAD_BONE")
	quit()
