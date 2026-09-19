extends RichTextLabel

func _ready() -> void:
	Autoload.new_log.connect(_on_new_log)

func _on_new_log(message: String) -> void:
	text = message
