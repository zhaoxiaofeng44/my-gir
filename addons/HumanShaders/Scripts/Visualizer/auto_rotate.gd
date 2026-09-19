extends Node3D

@export var use_right_click : bool = false
@export var rotation_sensitivity: float = 0.05
@export var world_env: WorldEnvironment
@export var rotate_sky: bool = false

@onready var ui: Control = %UI

# Internal tracking
var dragging: bool = false
var mouse_available : bool = true
var previous_mouse_position: Vector2

#func _ready() -> void:
#	if not Engine.is_editor_hint():
#		for i in ui.get_children():
#			if i is Control:
	#			i.mouse_entered.connect(on_ui_mouse_entered)
	#			i.mouse_exited.connect(on_ui_mouse_exited)
#				i.gui_input.connect(using_ui)

func _input(event: InputEvent) -> void:
	var mouse_click = MOUSE_BUTTON_LEFT
	if use_right_click:
		mouse_click = MOUSE_BUTTON_RIGHT
	
	if event is InputEventMouseMotion and dragging:
		# Calculate the drag distance
		var delta_x = event.relative.x
		# Rotate the model along the Y-axis
		rotation_degrees.y -= -delta_x * rotation_sensitivity
		# If true, rotate sky:
		if rotate_sky:
			world_env.environment.sky_rotation.y -= deg_to_rad(-delta_x * rotation_sensitivity)
	
	elif event is InputEventMouseButton and mouse_available:
		if event.button_index == mouse_click:
			if event.pressed and not is_mouse_over_ui():
				dragging = true
				previous_mouse_position = event.position
			else:
				dragging = false
	
func is_mouse_over_ui() -> bool:
	# Check if the mouse is interacting with the UI
	return get_viewport().gui_get_hovered_control() != null
