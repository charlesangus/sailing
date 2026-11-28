extends SceneTree

const STEP := 0.1

var _input: Node
var _boat: Node
var _failed := false


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	_failed = true


func _check(ok: bool, message: String) -> void:
	if not ok:
		_fail(message)


func _key(code: Key, pressed: bool) -> void:
	var e := InputEventKey.new()
	e.physical_keycode = code
	e.keycode = code
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _pad_button(index: JoyButton, pressed: bool) -> void:
	var e := InputEventJoypadButton.new()
	e.button_index = index
	e.pressed = pressed
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _pad_axis(axis: JoyAxis, value: float) -> void:
	var e := InputEventJoypadMotion.new()
	e.axis = axis
	e.axis_value = value
	Input.parse_input_event(e)
	Input.flush_buffered_events()


func _run(seconds: float) -> void:
	for i in int(round(seconds / STEP)):
		_input._process(STEP)


func _reset_controls() -> void:
	_boat.set_controls_from_dictionary({
		"tiller": 0.0, "mainsheet": 0.5, "traveler": 0.0, "backstay": 0.5,
		"jib_sheet_port": 0.5, "jib_sheet_starboard": 0.5, "jib_car": 0.5, "reef": 0.0,
	})


func _sheet_move(value_of: Callable, press: Callable, release: Callable, label: String, expect_sign: float) -> void:
	_reset_controls()
	var before: float = value_of.call()
	press.call()
	_run(1.0)
	release.call()
	var delta: float = float(value_of.call()) - before
	_check(delta * expect_sign > 0.1, "%s moved %f, expected sign %f" % [label, delta, expect_sign])


# A press and release two frames apart, left to the node's own _process: just-pressed actions are
# only seen within the frame of the press.
func _tap_key(code: Key) -> void:
	_key(code, true)
	await process_frame
	await process_frame
	_key(code, false)
	await process_frame
	await process_frame


# Steering reads no just-pressed state, so a tap is driven by hand for exact steps.
func _tap_steer(code: Key) -> void:
	_key(code, true)
	_input._process(STEP)
	_key(code, false)
	_input._process(STEP)


func _initialize() -> void:
	if not ClassDB.class_exists("SailBoat"):
		_fail("class SailBoat is not registered")
		quit(1)
		return

	_boat = ClassDB.instantiate("SailBoat")
	_boat.wind_speed_kn = 10.0
	_boat.wind_direction_deg = 180.0
	_boat.boat_dir = "res://data/boats/j80"
	root.add_child(_boat)
	if not _boat.is_active():
		_fail("boat did not load: " + _boat.get_last_error())
		quit(1)
		return

	var script: GDScript = load("res://scripts/boat_input.gd")
	_input = script.new()
	_input.boat_path = NodePath("../SailBoat")
	_boat.name = "SailBoat"
	root.add_child(_input)
	_input.set_process(false)
	_boat.set_process(false)

	# Wind from the south: heading east is starboard tack (awa > 0), heading west is port tack.
	_boat.reset_boat(Vector3.ZERO, 90.0, 3.0)
	_check(float(_boat.get_state()["awa_deg"]) > 0.0, "heading east should have awa > 0")

	# Keyboard.
	_sheet_move(func(): return _boat.tiller, func(): _key(KEY_D, true), func(): _key(KEY_D, false), "key D tiller", 1.0)
	_sheet_move(func(): return _boat.tiller, func(): _key(KEY_A, true), func(): _key(KEY_A, false), "key A tiller", -1.0)
	_sheet_move(func(): return _boat.tiller, func(): _key(KEY_RIGHT, true), func(): _key(KEY_RIGHT, false), "key Right tiller", 1.0)
	_sheet_move(func(): return _boat.mainsheet, func(): _key(KEY_W, true), func(): _key(KEY_W, false), "key W mainsheet", 1.0)
	_sheet_move(func(): return _boat.mainsheet, func(): _key(KEY_S, true), func(): _key(KEY_S, false), "key S mainsheet", -1.0)
	_sheet_move(func(): return _boat.traveler, func(): _key(KEY_F, true), func(): _key(KEY_F, false), "key F traveler leeward", 1.0)
	_sheet_move(func(): return _boat.traveler, func(): _key(KEY_R, true), func(): _key(KEY_R, false), "key R traveler windward", -1.0)
	_sheet_move(func(): return _boat.backstay, func(): _key(KEY_T, true), func(): _key(KEY_T, false), "key T backstay", 1.0)
	_sheet_move(func(): return _boat.backstay, func(): _key(KEY_G, true), func(): _key(KEY_G, false), "key G backstay", -1.0)
	_sheet_move(func(): return _boat.jib_car, func(): _key(KEY_X, true), func(): _key(KEY_X, false), "key X jib car aft", 1.0)
	_sheet_move(func(): return _boat.jib_car, func(): _key(KEY_Z, true), func(): _key(KEY_Z, false), "key Z jib car forward", -1.0)

	# Gamepad.
	_sheet_move(func(): return _boat.tiller, func(): _pad_axis(JOY_AXIS_LEFT_X, 0.8), func(): _pad_axis(JOY_AXIS_LEFT_X, 0.0), "stick right tiller", 1.0)
	_sheet_move(func(): return _boat.tiller, func(): _pad_axis(JOY_AXIS_LEFT_X, -0.8), func(): _pad_axis(JOY_AXIS_LEFT_X, 0.0), "stick left tiller", -1.0)
	_sheet_move(func(): return _boat.mainsheet, func(): _pad_axis(JOY_AXIS_TRIGGER_RIGHT, 1.0), func(): _pad_axis(JOY_AXIS_TRIGGER_RIGHT, 0.0), "RT mainsheet", 1.0)
	_sheet_move(func(): return _boat.mainsheet, func(): _pad_button(JOY_BUTTON_RIGHT_SHOULDER, true), func(): _pad_button(JOY_BUTTON_RIGHT_SHOULDER, false), "RB mainsheet", -1.0)
	_sheet_move(func(): return _boat.traveler, func(): _pad_button(JOY_BUTTON_DPAD_DOWN, true), func(): _pad_button(JOY_BUTTON_DPAD_DOWN, false), "dpad down traveler", 1.0)
	_sheet_move(func(): return _boat.traveler, func(): _pad_button(JOY_BUTTON_DPAD_UP, true), func(): _pad_button(JOY_BUTTON_DPAD_UP, false), "dpad up traveler", -1.0)
	_sheet_move(func(): return _boat.backstay, func(): _pad_axis(JOY_AXIS_RIGHT_Y, -1.0), func(): _pad_axis(JOY_AXIS_RIGHT_Y, 0.0), "right stick up backstay", 1.0)
	_sheet_move(func(): return _boat.backstay, func(): _pad_axis(JOY_AXIS_RIGHT_Y, 1.0), func(): _pad_axis(JOY_AXIS_RIGHT_Y, 0.0), "right stick down backstay", -1.0)
	_sheet_move(func(): return _boat.jib_car, func(): _pad_button(JOY_BUTTON_DPAD_RIGHT, true), func(): _pad_button(JOY_BUTTON_DPAD_RIGHT, false), "dpad right jib car", 1.0)
	_sheet_move(func(): return _boat.jib_car, func(): _pad_button(JOY_BUTTON_DPAD_LEFT, true), func(): _pad_button(JOY_BUTTON_DPAD_LEFT, false), "dpad left jib car", -1.0)

	# Helm: keys ramp to full in about 0.5 s and centre on release, the stick is proportional.
	_reset_controls()
	_key(KEY_D, true)
	_run(0.3)
	var partway: float = _boat.tiller
	_check(partway > 0.3 and partway < 0.9, "key helm should ramp, got %f" % partway)
	_run(0.5)
	_check(is_equal_approx(_boat.tiller, 1.0), "key helm should reach 1, got %f" % _boat.tiller)
	_key(KEY_D, false)
	_run(0.5)
	_check(is_equal_approx(_boat.tiller, 0.0), "key helm should self-centre, got %f" % _boat.tiller)
	_pad_axis(JOY_AXIS_LEFT_X, 0.5)
	_run(STEP)
	var scaled := (0.5 - 0.15) / (1.0 - 0.15)
	_check(absf(_boat.tiller - scaled) < 0.01, "stick helm should be proportional, got %f" % _boat.tiller)
	_pad_axis(JOY_AXIS_LEFT_X, 0.05)
	_run(1.0)
	_check(is_equal_approx(_boat.tiller, 0.0), "stick inside the deadzone should give no helm, got %f" % _boat.tiller)
	_pad_axis(JOY_AXIS_LEFT_X, 0.0)

	# The jib sheet drives the leeward side on both tacks.
	var jib_cases := [
		{"heading": 90.0, "leeward": "jib_sheet_port", "other": "jib_sheet_starboard"},
		{"heading": 270.0, "leeward": "jib_sheet_starboard", "other": "jib_sheet_port"},
	]
	for jib_case in jib_cases:
		_boat.reset_boat(Vector3.ZERO, jib_case["heading"], 3.0)
		for device in ["key", "pad"]:
			for direction in [1.0, -1.0]:
				_reset_controls()
				if device == "key":
					var code: Key = KEY_E if direction > 0.0 else KEY_Q
					_key(code, true)
					_run(1.0)
					_key(code, false)
				else:
					if direction > 0.0:
						_pad_axis(JOY_AXIS_TRIGGER_LEFT, 1.0)
						_run(1.0)
						_pad_axis(JOY_AXIS_TRIGGER_LEFT, 0.0)
					else:
						_pad_button(JOY_BUTTON_LEFT_SHOULDER, true)
						_run(1.0)
						_pad_button(JOY_BUTTON_LEFT_SHOULDER, false)
				var moved: float = float(_boat.get(jib_case["leeward"])) - 0.5
				var untouched: float = float(_boat.get(jib_case["other"])) - 0.5
				_check(moved * direction > 0.1, "%s %s dir %f: leeward moved %f" % [device, jib_case["leeward"], direction, moved])
				_check(is_zero_approx(untouched), "%s heading %f: windward sheet moved %f" % [device, jib_case["heading"], untouched])

	await process_frame
	await process_frame
	# Toggles and signals fire once per press.
	var counts := {"camera": 0, "mark": 0, "settings": 0}
	_input.camera_switch_requested.connect(func(): counts["camera"] += 1)
	_input.mark_drop_requested.connect(func(): counts["mark"] += 1)
	_input.settings_toggled.connect(func(): counts["settings"] += 1)
	var toggles := [
		[func(): _key(KEY_B, true), func(): _key(KEY_B, false), "key reef"],
		[func(): _pad_button(JOY_BUTTON_A, true), func(): _pad_button(JOY_BUTTON_A, false), "pad reef"],
	]
	_reset_controls()
	for toggle in toggles:
		var before: float = _boat.reef
		toggle[0].call()
		await process_frame
		_input._process(STEP)
		_check(not is_equal_approx(_boat.reef, before), "%s did not toggle" % toggle[2])
		var toggled: float = _boat.reef
		await process_frame
		_input._process(STEP)
		_check(is_equal_approx(_boat.reef, toggled), "%s toggled again while held" % toggle[2])
		toggle[1].call()
		await process_frame
		_input._process(STEP)
	var signal_cases := [
		["camera", KEY_C, JOY_BUTTON_Y],
		["mark", KEY_SPACE, JOY_BUTTON_X],
		["settings", KEY_ESCAPE, JOY_BUTTON_START],
	]
	for signal_case in signal_cases:
		for device in ["key", "pad"]:
			var before_count: int = counts[signal_case[0]]
			if device == "key":
				_key(signal_case[1], true)
			else:
				_pad_button(signal_case[2], true)
			await process_frame
			_input._process(STEP)
			_input._process(STEP)
			if device == "key":
				_key(signal_case[1], false)
			else:
				_pad_button(signal_case[2], false)
			await process_frame
			_input._process(STEP)
			_check(counts[signal_case[0]] == before_count + 1, "%s %s fired %d times" % [signal_case[0], device, counts[signal_case[0]] - before_count])

	# Helm hold: a tap steps the tiller by a fixed amount, the held helm keeps it, and releasing the
	# hold lets it self-centre again.
	_reset_controls()
	await _tap_key(KEY_V)
	_check(_input.helm_hold, "key V should hold the helm")
	_input.set_process(false)
	_tap_steer(KEY_D)
	_tap_steer(KEY_D)
	_run(1.0)
	_check(is_equal_approx(_boat.tiller, 2.0 * BoatInput.HELM_TAP_STEP), "two held taps should give %f, got %f" % [2.0 * BoatInput.HELM_TAP_STEP, _boat.tiller])
	_tap_steer(KEY_A)
	_check(is_equal_approx(_boat.tiller, BoatInput.HELM_TAP_STEP), "a tap to port should step back, got %f" % _boat.tiller)
	_input.set_process(true)
	await _tap_key(KEY_V)
	_check(not _input.helm_hold, "key V again should release the hold")
	_run(0.5)
	_check(is_zero_approx(_boat.tiller), "the released hold should self-centre, got %f" % _boat.tiller)

	# With a panel open the helm centres, so the boat does not circle under it.
	_reset_controls()
	_boat.tiller = 1.0
	_pad_axis(JOY_AXIS_LEFT_X, 1.0)
	_input.controls_enabled = false
	_run(0.5)
	_check(is_zero_approx(_boat.tiller), "the helm should centre while controls are disabled, got %f" % _boat.tiller)
	_input.controls_enabled = true
	_pad_axis(JOY_AXIS_LEFT_X, 0.0)

	# A pad button held through an unplug is released, so its key presses fresh afterwards.
	_pad_button(JOY_BUTTON_Y, true)
	await process_frame
	await process_frame
	var camera_count: int = counts["camera"]
	Input.joy_connection_changed.emit(0, false)
	_check(not Input.is_action_pressed(&"camera_switch"), "camera_switch stayed held after the unplug")
	await _tap_key(KEY_C)
	_check(counts["camera"] == camera_count + 1, "key C after the unplug fired %d times" % (counts["camera"] - camera_count))
	_pad_button(JOY_BUTTON_Y, false)

	# Unplugging a pad with a trigger held stops that input and leaves the keyboard working.
	_reset_controls()
	_pad_axis(JOY_AXIS_TRIGGER_RIGHT, 1.0)
	_run(0.5)
	var held: float = _boat.mainsheet
	_check(held > 0.5, "trigger should trim the main before the unplug")
	Input.joy_connection_changed.emit(0, false)
	_run(0.5)
	_check(is_equal_approx(_boat.mainsheet, held), "main kept moving after the pad was unplugged")
	_key(KEY_S, true)
	_run(1.0)
	_key(KEY_S, false)
	_check(_boat.mainsheet < held - 0.1, "keyboard did not work after the pad was unplugged")
	_pad_axis(JOY_AXIS_TRIGGER_RIGHT, 0.0)

	_input.free()
	_boat.free()
	if _failed:
		quit(1)
		return
	print("PASS")
	quit(0)
