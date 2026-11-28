extends SceneTree

const FRAMES := 3
const TOLERANCE_M := 1.0
const WINDWARD_M := 926.0

var _failed := false


func _check(ok: bool, message: String) -> void:
	if not ok:
		printerr("FAIL: " + message)
		_failed = true


func _key(code: Key, pressed: bool) -> void:
	var e := InputEventKey.new()
	e.physical_keycode = code
	e.keycode = code
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _pad(pressed: bool) -> void:
	var e := InputEventJoypadButton.new()
	e.button_index = JOY_BUTTON_X
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _frames() -> void:
	for i in FRAMES:
		await process_frame


func _buoys(marks: Node) -> Array[Node]:
	var found: Array[Node] = []
	for child in marks.get_children():
		if child.has_node("Body"):
			found.append(child)
	return found


func _tap(press: Callable, release: Callable) -> void:
	press.call()
	await _frames()
	release.call()
	await _frames()


func _initialize() -> void:
	var main: Node = (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(main)
	await _frames()
	var boat: Node3D = main.get_node("Boat")
	var marks = main.get_node("Marks")
	_check(_buoys(marks).size() == 0, "buoys exist before any drop")

	var at := Vector3(120.0, 0.0, -340.0)
	var tap_key := func(): await _tap(func(): _key(KEY_SPACE, true), func(): _key(KEY_SPACE, false))
	var tap_pad := func(): await _tap(func(): _pad(true), func(): _pad(false))
	var taps := [tap_key, tap_pad]
	var count := 0
	for from_deg in [0.0, 135.0]:
		boat.reset_boat(at, 90.0)
		boat.wind_direction_deg = from_deg
		var bearing := deg_to_rad(from_deg)
		var expect_windward := at + Vector3(sin(bearing), 0.0, -cos(bearing)) * WINDWARD_M
		for tap in taps:
			await tap.call()
			count += 1
			var buoys := _buoys(marks)
			_check(buoys.size() == 2, "wind %.0f drop %d: %d buoys, want 2" % [from_deg, count, buoys.size()])
			_check(marks.windward != null and marks.leeward != null, "marks not assigned")
			if marks.windward == null or marks.leeward == null:
				continue
			var w: Vector3 = marks.windward.global_position
			var l: Vector3 = marks.leeward.global_position
			_check(w.distance_to(expect_windward) <= TOLERANCE_M,
				"wind %.0f: windward at %s, want %s" % [from_deg, w, expect_windward])
			_check(absf(Vector2(w.x - at.x, w.z - at.z).length() - WINDWARD_M) <= TOLERANCE_M,
				"wind %.0f: windward is not 926 m away" % from_deg)
			_check(Vector2(l.x - at.x, l.z - at.z).length() <= TOLERANCE_M, "wind %.0f: leeward not at the boat: %s" % [from_deg, l])
			_check(is_zero_approx(w.y) and is_zero_approx(l.y), "marks are not at y = 0")
			await process_frame
			_check(_buoys(marks).size() == 2, "old buoys linger after replacement")

	quit(1 if _failed else 0)
	if not _failed:
		print("PASS")
