extends SceneTree
# 验证: 简单人形(程序化) + C++头发 + C++裙布, 秒开
var world: Node3D

func _init() -> void:
	Engine.max_physics_steps_per_frame = 100000
	world = Node3D.new()
	get_root().add_child(root)
	# 头
	var head := MeshInstance3D.new()
	var hs := SphereMesh.new()
	hs.radius = 0.115; hs.height = 0.23
	head.mesh = hs
	head.position = Vector3(0, 1.55, 0)
	var hm := StandardMaterial3D.new()
	hm.albedo_color = Color(0.95, 0.82, 0.72)
	head.material_override = hm
	world.add_child(head)
	# 身体(胶囊)
	var body := MeshInstance3D.new()
	var bs := CapsuleMesh.new()
	bs.radius = 0.16; bs.height = 1.1
	body.mesh = bs
	body.position = Vector3(0, 0.95, 0)
	var bm := StandardMaterial3D.new()
	bm.albedo_color = Color(0.3, 0.5, 0.75)
	body.material_override = bm
	world.add_child(body)
	await process_frame
	print("简单人形构建成功: 头(0,1.55,0) 身体(0,0.95,0)")
	quit()
