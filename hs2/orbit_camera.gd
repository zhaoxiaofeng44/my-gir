# orbit_camera.gd — 捏人视口相机控制器
# 鼠标：  左键拖拽=旋转 | 右键/中键拖拽=平移 | 滚轮=缩放 | 双指捏合=缩放
# 触控板：双指滑动=平移（按住 Shift 或 Cmd 时为缩放）| 捏合=缩放
# 触摸屏：单指拖拽=旋转 | 双指拖拽=平移 | 双指捏合=缩放
# 键盘：  R=复位视角  |  F=对焦到角色
extends Camera3D

@export var focus := Vector3(0.0, 1.0, 0.0)
@export var distance := 3.2
@export var yaw := 0.0          # 绕竖直轴，0 = 从角色正面(+Z)看
@export var pitch := -0.05      # 俯仰，正=从上往下看
@export var min_distance := 0.12
@export var max_distance := 10.0
@export var orbit_speed := 0.007
@export var pan_speed := 0.0016
@export var zoom_step := 1.10
@export var invert_y := false

var _home := {}
var _orbiting := false
var _panning := false
var _touches := {}              # index -> Vector2
var _pinch_dist := -1.0
var _two_finger_pan := Vector2.ZERO
var _auto_frame := true

func _ready() -> void:
	_home = {"focus": focus, "distance": distance, "yaw": yaw, "pitch": pitch}
	_apply()

## 若外部（脚本/UI）移动过焦点，可调用此函数复位
func reset_view() -> void:
	focus = _home["focus"]
	distance = _home["distance"]
	yaw = _home["yaw"]
	pitch = _home["pitch"]
	_apply()

## 自动把角色框进画面（首次加载时用）
func frame_aabb(a: AABB) -> void:
	if a.size == Vector3.ZERO:
		return
	focus = a.get_center() + Vector3(0, a.size.y * 0.02, 0)
	distance = clampf(a.size.y * 2.0, min_distance, max_distance)
	pitch = -0.05
	yaw = 0.0
	_home = {"focus": focus, "distance": distance, "yaw": yaw, "pitch": pitch}
	_apply()

func _apply() -> void:
	pitch = clampf(pitch, deg_to_rad(-89.0), deg_to_rad(89.0))
	distance = clampf(distance, min_distance, max_distance)
	var cp := cos(pitch)
	var dir := Vector3(sin(yaw) * cp, sin(pitch), cos(yaw) * cp)
	global_position = focus + dir * distance
	look_at(focus, Vector3.UP)

func _zoom(factor: float) -> void:
	distance = clampf(distance * factor, min_distance, max_distance)
	_apply()

func _orbit(rel: Vector2) -> void:
	yaw -= rel.x * orbit_speed
	pitch += (rel.y * orbit_speed) * (-1.0 if invert_y else 1.0)
	_apply()

func _pan(rel: Vector2) -> void:
	# 沿相机右向/上向平移焦点，位移随距离缩放（越近越慢）
	var s := pan_speed * maxf(distance, 0.05)
	focus += (-global_transform.basis.x * rel.x + global_transform.basis.y * rel.y) * s
	_apply()

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.pressed:
			match mb.button_index:
				MOUSE_BUTTON_WHEEL_UP:
					_zoom(1.0 / zoom_step)
				MOUSE_BUTTON_WHEEL_DOWN:
					_zoom(zoom_step)
				MOUSE_BUTTON_LEFT:
					_orbiting = true
				MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE:
					_panning = true
		else:
			match mb.button_index:
				MOUSE_BUTTON_LEFT:
					_orbiting = false
				MOUSE_BUTTON_RIGHT, MOUSE_BUTTON_MIDDLE:
					_panning = false
	elif event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if _orbiting:
			_orbit(mm.relative)
		elif _panning:
			_pan(mm.relative)
	elif event is InputEventMagnifyGesture:
		# macOS 触控板双指捏合
		var mg := event as InputEventMagnifyGesture
		_zoom(1.0 / maxf(mg.factor, 0.01))
	elif event is InputEventPanGesture:
		# macOS 触控板双指滑动：默认平移；按住 Shift/Cmd 时缩放
		var pg := event as InputEventPanGesture
		if Input.is_key_pressed(KEY_SHIFT) or Input.is_key_pressed(KEY_META):
			_zoom(1.0 + pg.delta.y * 0.01)
		else:
			_pan(pg.delta * 4.0)
	elif event is InputEventScreenTouch:
		var st := event as InputEventScreenTouch
		if st.pressed:
			_touches[st.index] = st.position
		else:
			_touches.erase(st.index)
		_pinch_dist = -1.0
		_two_finger_pan = Vector2.ZERO
	elif event is InputEventScreenDrag:
		var sd := event as InputEventScreenDrag
		_touches[sd.index] = sd.position
		if _touches.size() == 1:
			_orbit(sd.relative)
		elif _touches.size() >= 2:
			var pts: Array = _touches.values()
			var p0: Vector2 = pts[0]
			var p1: Vector2 = pts[1]
			var d := p0.distance_to(p1)
			var mid := (p0 + p1) * 0.5
			if _pinch_dist > 0.0 and d > 1.0:
				_zoom(_pinch_dist / d)
			_pinch_dist = d
			if _two_finger_pan != Vector2.ZERO:
				_pan(mid - _two_finger_pan)
			_two_finger_pan = mid
	elif event is InputEventKey and event.pressed and not event.echo:
		match (event as InputEventKey).keycode:
			KEY_R:
				reset_view()
			KEY_F:
				pass   # 由场景脚本处理（对焦角色）
