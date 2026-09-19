extends Camera3D

# Variables to define FOV limits and sensitivity
@export var min_fov: float = 10.0
@export var max_fov: float = 75.0
@export var default_fov: float = 25.0
@export var zoom_sensitivity: float = 2.0
@export var movement_sensitivity: float = 2.0

var dragging: bool = false
var previous_mouse_position: Vector2
var default_pos: Vector3
var tween: Tween

func _ready() -> void:
	default_pos = position
	fov = default_fov
	
	
#func transition(property: String, value: float) -> float:
#	tween.tween_property(self, property, )

func _input(event: InputEvent) -> void:
	
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			tween = create_tween().bind_node(self).set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
			tween.tween_property(
				self, 
				"fov",
				clamp(fov - zoom_sensitivity, min_fov, max_fov),
				0.1
			)
			
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			tween = create_tween().bind_node(self).set_trans(Tween.TRANS_QUAD).set_ease(Tween.EASE_OUT)
			tween.tween_property(
				self, 
				"fov",
				clamp(fov + zoom_sensitivity, min_fov, max_fov),
				0.1
			)
			
		if event.button_index == MOUSE_BUTTON_MIDDLE:
			if event.pressed:
				dragging = true
				previous_mouse_position = event.position
			else:
				dragging = false
		
	elif event is InputEventMouseMotion and dragging:
					# Calculate the drag distance
			var delta_xy = Vector3(-event.relative.x, event.relative.y, 0)
			# Rotate the model along the Y-axis
			position -= -delta_xy * movement_sensitivity * .0001


func _on_reset_camera_pressed() -> void:
	position = default_pos
	fov = default_fov
	Autoload.log_message("Reset Camera Applied")
