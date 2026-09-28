extends SceneTree
# 布料视觉验收截图: 落定贴面 / 强风 / 拖拽碰撞体穿挂帘 / 3布料互撞 / 胶囊运动
# 用法: godot --path . -s tools/shot_cloth.gd (需窗口渲染)

var scene: Node

func _init() -> void:
	scene = (load("res://scenes/xpbd_cloth_test.tscn") as PackedScene).instantiate()
	get_root().add_child(scene)
	await process_frame
	for shot in [
		{"tag": "settle", "steps": 240, "mode": 0, "wind": 0.0, "moving": false, "n": 2, "blob": null},
		{"tag": "wind", "steps": 90, "mode": 0, "wind": 14.0, "moving": false, "n": 2, "blob": null},
		{"tag": "drag_curtain", "steps": 300, "mode": 2, "wind": 0.0, "moving": false, "n": 2, "blob": Vector3(-1.2, 1.1, -0.6)},
		{"tag": "inter3", "steps": 300, "mode": 0, "wind": 3.0, "moving": false, "n": 3, "blob": Vector3(0.2, 0.9, 0.3)},
		{"tag": "capsule", "steps": 260, "mode": 1, "wind": 6.0, "moving": true, "n": 2, "blob": null},
		{"tag": "cover5", "steps": 700, "mode": 0, "wind": 0.0, "moving": false, "n": 5, "blob": null},
	]:
		if not scene.is_inside_tree():
			continue
		scene.collider_mode = shot["mode"]
		scene.wind_strength = shot["wind"]
		scene.blob_moving = shot["moving"]
		scene.blob_center = Vector3(0, 0.2, 0)
		scene.blob_yaw = 0.0
		scene.cloth_count = shot["n"]
		scene._build_blob()
		scene._rebuild_cloths()
		for _s in shot["steps"]:
			await process_frame
		if shot["blob"] != null:
			# 走场景拖拽限速通道, 与真实用户操作一致
			scene.drag_collider = true
			scene.drag_pos = shot["blob"]
			for s in 160:
				await process_frame
			scene.drag_collider = false
		await process_frame
		var img := root.get_texture().get_image()
		var abs_path := ProjectSettings.globalize_path("res://xpbd_shot_%s.png" % shot["tag"])
		var err := img.save_png(abs_path)
		print("saved xpbd_shot_%s.png err=%d" % [shot["tag"], err])
	quit()
