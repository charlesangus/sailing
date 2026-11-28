extends SceneTree

const FRAMES := 4

var _failed := false


func _fail(message: String) -> void:
	printerr("FAIL: " + message)
	_failed = true


func _check(ok: bool, message: String) -> void:
	if not ok:
		_fail(message)


func _finite(t: Transform3D) -> bool:
	return t.origin.is_finite() and t.basis.x.is_finite() and t.basis.y.is_finite() and t.basis.z.is_finite()


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


func _frames(count: int) -> void:
	for i in count:
		await process_frame


func _tap(press: Callable, release: Callable) -> void:
	press.call()
	await _frames(2)
	release.call()
	await _frames(2)


func _initialize() -> void:
	var main = (load("res://scenes/main.tscn") as PackedScene).instantiate()
	root.add_child(main)
	await _frames(FRAMES)
	await _run(main)
	if _failed:
		quit(1)
		return
	print("PASS")
	quit(0)


func _run(main: Node) -> void:
	var rig: CameraRig = main.get_node("CameraRig")
	var boat: Node3D = main.get_node("Boat")
	var input: BoatInput = main.get_node("Boat/BoatInput")
	var camera: Camera3D = main.get_node("Camera")
	var visual: BoatVisual = boat.get_node("Visual")
	if rig.helm_camera == null or not visual.is_built():
		_fail("the helm camera or the boat visual was not built")
		return

	_check(rig.mode == CameraRig.Mode.CHASE, "the rig should start in chase mode")
	_check(_finite(camera.global_transform), "chase transform is not finite")
	var forward := -boat.global_transform.basis.z
	forward.y = 0.0
	forward = forward.normalized()
	var offset := camera.global_position - boat.global_position
	_check(offset.dot(forward) < -0.9 * rig.chase_distance_m, "chase camera is not behind the boat: %s" % offset)
	_check(absf(offset.y - (rig.chase_height_m)) < 0.5, "chase height off: %f" % offset.y)
	_check(absf(camera.global_transform.basis.x.y) < 1e-4, "chase horizon is not level")
	_check(camera.is_current(), "chase camera is not current")
	_check(root.get_camera_3d() == camera, "the viewport camera is not the chase camera")

	# Heel does not tilt the chase view.
	boat.rotate_object_local(Vector3.FORWARD, 0.4)
	await _frames(2)
	_check(absf(camera.global_transform.basis.x.y) < 1e-4, "chase horizon follows heel")
	boat.global_transform = Transform3D(Basis.IDENTITY, boat.global_position)

	# Keyboard cycles chase -> orbit -> helm -> chase.
	await _tap(func(): _key(KEY_C, true), func(): _key(KEY_C, false))
	_check(rig.mode == CameraRig.Mode.ORBIT, "key C should enter orbit, mode %d" % rig.mode)
	await _check_orbit(rig, boat, camera)
	await _tap(func(): _key(KEY_C, true), func(): _key(KEY_C, false))
	_check(rig.mode == CameraRig.Mode.HELM, "key C should enter helm, mode %d" % rig.mode)
	_check_helm(rig, visual)
	await _tap(func(): _key(KEY_C, true), func(): _key(KEY_C, false))
	_check(rig.mode == CameraRig.Mode.CHASE, "key C should return to chase")

	# The gamepad cycles the same way.
	for expected in [CameraRig.Mode.ORBIT, CameraRig.Mode.HELM, CameraRig.Mode.CHASE]:
		await _tap(func(): _pad_button(JOY_BUTTON_Y, true), func(): _pad_button(JOY_BUTTON_Y, false))
		_check(rig.mode == expected, "pad Y should reach mode %d, got %d" % [expected, rig.mode])
		_check(_finite(rig.current_camera().global_transform), "mode %d transform is not finite" % expected)
		_check(rig.current_camera().is_current(), "mode %d camera is not current" % expected)

	# The right stick orbits in orbit mode and trims the backstay otherwise.
	boat.backstay = 0.5
	await _tap(func(): _pad_axis(JOY_AXIS_RIGHT_Y, -1.0), func(): _pad_axis(JOY_AXIS_RIGHT_Y, 0.0))
	_check(boat.backstay > 0.5, "the right stick should trim the backstay outside orbit mode")
	rig.set_mode(CameraRig.Mode.ORBIT)
	boat.backstay = 0.5
	var pitch_before := rig.orbit_pitch
	await _tap(func(): _pad_axis(JOY_AXIS_RIGHT_Y, -1.0), func(): _pad_axis(JOY_AXIS_RIGHT_Y, 0.0))
	_check(is_equal_approx(boat.backstay, 0.5), "the right stick should not trim the backstay in orbit mode")
	_check(rig.orbit_pitch > pitch_before, "the right stick should raise the orbit camera")
	await _tap(func(): _key(KEY_T, true), func(): _key(KEY_T, false))
	_check(boat.backstay > 0.5, "the backstay key should work in orbit mode")

	# The stick clicks zoom; the left stick, which steers, no longer does.
	var distance_before := rig.orbit_distance
	await _tap(func(): _pad_button(JOY_BUTTON_RIGHT_STICK, true), func(): _pad_button(JOY_BUTTON_RIGHT_STICK, false))
	_check(rig.orbit_distance < distance_before, "R3 should zoom in: %f -> %f" % [distance_before, rig.orbit_distance])
	distance_before = rig.orbit_distance
	await _tap(func(): _pad_button(JOY_BUTTON_LEFT_STICK, true), func(): _pad_button(JOY_BUTTON_LEFT_STICK, false))
	_check(rig.orbit_distance > distance_before, "L3 should zoom out: %f -> %f" % [distance_before, rig.orbit_distance])
	distance_before = rig.orbit_distance
	await _tap(func(): _pad_axis(JOY_AXIS_LEFT_Y, 1.0), func(): _pad_axis(JOY_AXIS_LEFT_Y, 0.0))
	_check(is_equal_approx(rig.orbit_distance, distance_before), "the left stick should not zoom")

	# In helm mode the right stick looks around instead of trimming the backstay.
	rig.set_mode(CameraRig.Mode.HELM)
	boat.backstay = 0.5
	await _tap(func(): _pad_axis(JOY_AXIS_RIGHT_X, 1.0), func(): _pad_axis(JOY_AXIS_RIGHT_X, 0.0))
	_check(rig._look_yaw < 0.0, "the right stick right should look to starboard, yaw %f" % rig._look_yaw)
	await _tap(func(): _pad_axis(JOY_AXIS_RIGHT_Y, -1.0), func(): _pad_axis(JOY_AXIS_RIGHT_Y, 0.0))
	_check(rig._look_pitch > 0.0, "the right stick up should look up, pitch %f" % rig._look_pitch)
	_check(is_equal_approx(boat.backstay, 0.5), "the right stick should not trim the backstay in helm mode")
	var look := -rig.helm_camera.basis.z
	_check(look.y < 0.0, "a look to starboard should turn the helm view towards hull -y, got %s" % look)
	await _tap(func(): _key(KEY_T, true), func(): _key(KEY_T, false))
	_check(boat.backstay > 0.5, "the backstay key should work in helm mode")
	rig.set_mode(CameraRig.Mode.CHASE)
	await _frames(2)


func _check_orbit(rig: CameraRig, boat: Node3D, camera: Camera3D) -> void:
	await _frames(2)
	_check(_finite(camera.global_transform), "orbit transform is not finite")
	var target := boat.global_position + Vector3.UP * rig.orbit_target_height_m
	_check(absf(camera.global_position.distance_to(target) - rig.orbit_distance_m) < 0.05,
			"orbit distance is %f, want %f" % [camera.global_position.distance_to(target), rig.orbit_distance_m])
	_check(camera.is_current(), "orbit camera is not current")

	# Extremes of pitch and zoom keep the camera above the water and inside the limits.
	await _tap(func(): _key(KEY_K, true), func(): _key(KEY_K, false))
	for i in 200:
		rig._orbit_by(0.1, -0.1)
		rig._zoom_by(0.5)
	rig._update_cameras(0.0)
	_check(camera.global_position.y > 0.0, "orbit camera went under the water")
	_check(rig.orbit_distance >= rig.orbit_min_distance_m - 1e-4, "orbit zoomed in past its limit")
	for i in 200:
		rig._orbit_by(0.1, 0.1)
		rig._zoom_by(2.0)
	rig._update_cameras(0.0)
	_check(rig.orbit_distance <= rig.orbit_max_distance_m + 1e-4, "orbit zoomed out past its limit")
	_check(rig.orbit_pitch <= deg_to_rad(rig.orbit_max_pitch_deg) + 1e-6, "orbit pitch past its limit")
	_check(_finite(camera.global_transform), "extreme orbit transform is not finite")


func _check_helm(rig: CameraRig, visual: BoatVisual) -> void:
	var cam := rig.helm_camera
	_check(_finite(cam.global_transform), "helm transform is not finite")
	_check(cam.is_current(), "helm camera is not current")
	_check(cam.get_parent() == visual.hull_frame, "the helm camera should ride on the hull")
	var box := AABB()
	var first := true
	for node in visual.hull_frame.get_children():
		if node is MeshInstance3D:
			var part: AABB = node.transform * node.mesh.get_aabb()
			box = part if first else box.merge(part)
			first = false
	_check(box.has_point(cam.position), "the helm eye %s is outside the boat's bounds %s" % [cam.position, box])
	_check(cam.position.x < 0.0, "the helm eye should be aft of midships")
	var look := -cam.global_transform.basis.z
	var bow: Vector3 = -visual.get_parent().global_transform.basis.z
	_check(look.dot(bow) > 0.9, "the helm camera should look forward")
	_check(look.y < 0.0, "the helm camera should look slightly down")
