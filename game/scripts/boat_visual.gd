class_name BoatVisual
extends Node3D
## Placeholder visuals for a SailBoat parent: the dossier hull and foils, a procedural rig and flat
## sails, posed every frame from SailBoat.get_state().
##
## Every mesh is built in the hull frame of the dossier (x forward, y to port, z up, origin at DWL
## midship) under one HullFrame node. HullFrame carries the hull-to-Godot axis change of
## gdext/src/frames.hpp and the offset to the CG, which the SailBoat node follows and which moves
## with the crew, so no vertex is converted by hand and rotations about the hull z axis are
## rotations about the hull's vertical.

const HULL_COLOUR := Color(0.93, 0.94, 0.95)
const APPENDAGE_COLOUR := Color(0.12, 0.13, 0.15)
const SPAR_COLOUR := Color(0.55, 0.57, 0.6)
const SAIL_COLOUR := Color(0.95, 0.93, 0.86)
const LUFF_COLOUR := Color(0.95, 0.72, 0.55)

const MAST_RADIUS := 0.055
const BOOM_RADIUS := 0.047
const TILLER_LENGTH := 0.9
const TILLER_HEIGHT := 0.75
const MAIN_ROACH_M := 0.25
const ROACH_POINTS := 4
const SWING_ITERATIONS := 30

## Hull frame x, y, z in Godot node-local axes, the body map of frames.hpp.
const HULL_TO_LOCAL := Basis(Vector3(0.0, 0.0, -1.0), Vector3(-1.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0))

var hull_frame: Node3D
var hull_mesh: MeshInstance3D
var keel_mesh: MeshInstance3D
var rudder_pivot: Node3D
var rudder_mesh: MeshInstance3D
var tiller_mesh: MeshInstance3D
var mast_mesh: MeshInstance3D
var boom: Node3D
var main_sail: MeshInstance3D
var jib_sail: MeshInstance3D
var last_error := ""

var _boat: Node3D
var _main_material: StandardMaterial3D
var _jib_material: StandardMaterial3D
var _main_geometry := {}
var _jib_geometry := {}


func _ready() -> void:
	_boat = get_parent() as Node3D
	if _boat == null or not _boat.has_method("get_state"):
		_fail("BoatVisual needs a SailBoat parent")
		return
	if build():
		update_from_state()


func _process(_delta: float) -> void:
	if hull_frame != null:
		update_from_state()


func is_built() -> bool:
	return hull_frame != null


func build() -> bool:
	var boat_dir: String = _boat.boat_dir
	var hydro := _read_json(boat_dir.path_join("hull_hydro.json"))
	var rig := _read_json(boat_dir.path_join("rig.json"))
	var sails := _read_json(boat_dir.path_join("sails.json"))
	var appendages := _read_json(boat_dir.path_join("appendages.json"))
	if hydro.is_empty() or rig.is_empty() or sails.is_empty() or appendages.is_empty():
		return false
	var hull_objects := _read_obj(boat_dir.path_join("hull_visual.obj"))
	var foil_objects := _read_obj(boat_dir.path_join("appendages.obj"))
	if hull_objects.is_empty() or not foil_objects.has("keel") or not foil_objects.has("rudder"):
		return _fail("the boat meshes could not be read from " + boat_dir)

	# Rig and sail positions are in the provisional frame, which the dossier maps to the hull frame.
	var shift := Vector3(float(hydro["transform"]["stem_x_in_body"]), 0.0,
			float(hydro["transform"]["flotation_z_in_body"]))
	var stock := _vec3(appendages["foils"]["rudder"]["stock_position"]["value"])

	hull_frame = Node3D.new()
	hull_frame.name = "HullFrame"
	hull_frame.basis = HULL_TO_LOCAL
	add_child(hull_frame)

	var hull_material := _material(HULL_COLOUR)
	var dark := _material(APPENDAGE_COLOUR)
	var grey := _material(SPAR_COLOUR)

	hull_mesh = _mesh_node("Hull", _mesh_from_obj(hull_objects.values()[0]), hull_material)
	hull_frame.add_child(hull_mesh)
	keel_mesh = _mesh_node("Keel", _mesh_from_obj(foil_objects["keel"]), dark)
	hull_frame.add_child(keel_mesh)

	rudder_pivot = Node3D.new()
	rudder_pivot.name = "RudderPivot"
	rudder_pivot.position = stock
	hull_frame.add_child(rudder_pivot)
	rudder_mesh = _mesh_node("Rudder", _mesh_from_obj(foil_objects["rudder"]), dark)
	rudder_mesh.position = -stock
	rudder_pivot.add_child(rudder_mesh)
	tiller_mesh = _box_node("Tiller", Vector3(TILLER_LENGTH, 0.04, 0.04), dark)
	tiller_mesh.position = Vector3(TILLER_LENGTH * 0.5, 0.0, TILLER_HEIGHT)
	rudder_pivot.add_child(tiller_mesh)

	var mast_base := _vec3(rig["mast_axis_at_sheer"]["value"]) + shift
	var mast_top := Vector3(mast_base.x, 0.0, float(rig["mast_top_above_water"]["value"]))
	mast_mesh = _cylinder_node("Mast", mast_base, mast_top, MAST_RADIUS, grey)
	hull_frame.add_child(mast_mesh)

	var main: Dictionary = sails["main"]
	_main_geometry = {
		"tack": _vec3(main["tack"]["value"]) + shift,
		"head": _vec3(main["head"]["value"]) + shift,
		"clew": _vec3(main["clew"]["value"]) + shift,
		"roach": MAIN_ROACH_M,
	}
	var jib: Dictionary = sails["jib"]
	_jib_geometry = {
		"tack": _vec3(jib["tack"]["value"]) + shift,
		"head": _vec3(jib["head"]["value"]) + shift,
		"clew": _vec3(jib["clew"]["value"]) + shift,
		"roach": 0.0,
	}

	boom = Node3D.new()
	boom.name = "Boom"
	boom.position = _main_geometry["tack"]
	hull_frame.add_child(boom)
	var boom_vector: Vector3 = _main_geometry["clew"] - _main_geometry["tack"]
	var boom_mesh := _cylinder_node("BoomSpar", Vector3.ZERO, boom_vector, BOOM_RADIUS, grey)
	boom.add_child(boom_mesh)

	_main_material = _sail_material()
	_jib_material = _sail_material()
	main_sail = _sail_node("MainSail", _main_geometry, _main_material)
	jib_sail = _sail_node("JibSail", _jib_geometry, _jib_material)
	hull_frame.add_child(main_sail)
	hull_frame.add_child(jib_sail)
	return true


## Poses the hull frame, boom, rudder and sails from the boat's current state.
func update_from_state() -> void:
	var state: Dictionary = _boat.get_state()
	if not state.get("active", false):
		return
	hull_frame.position = state["hull_origin_local"]
	rudder_pivot.basis = Basis(Vector3(0.0, 0.0, 1.0), deg_to_rad(float(state["rudder_angle_deg"])))

	var main_swing := _swing_for_chord_angle(_main_geometry, deg_to_rad(float(state["main_angle_deg"])))
	var boom_swing := _swing_for_chord_angle(_main_geometry, deg_to_rad(float(state["boom_angle_deg"])))
	boom.basis = Basis(Vector3(0.0, 0.0, 1.0), boom_swing)
	main_sail.position = _main_geometry["tack"]
	main_sail.basis = Basis(_luff_axis(_main_geometry), main_swing)
	var jib_swing := _swing_for_chord_angle(_jib_geometry, deg_to_rad(float(state["jib_angle_deg"])))
	jib_sail.position = _jib_geometry["tack"]
	jib_sail.basis = Basis(_luff_axis(_jib_geometry), jib_swing)

	_main_material.albedo_color = LUFF_COLOUR if state.get("main_luffing", false) else SAIL_COLOUR
	_jib_material.albedo_color = LUFF_COLOUR if state.get("jib_luffing", false) else SAIL_COLOUR


# The sail's chord angle is measured in the body xy plane from dead aft, positive with the clew to
# port. A rotation about the luff axis moves the clew along a cone, so for a raked luff the rotation
# that gives a wanted projected angle is found by iteration. About a vertical luff it is exactly
# minus the angle: positive rotation about hull z carries the clew to starboard.
func _swing_for_chord_angle(geometry: Dictionary, wanted: float) -> float:
	var axis := _luff_axis(geometry)
	var chord: Vector3 = geometry["clew"] - geometry["tack"]
	var swing := -wanted
	for i in SWING_ITERATIONS:
		var clew := Basis(axis, swing) * chord
		var error := wanted - atan2(clew.y, -clew.x)
		if absf(error) < 1e-6:
			break
		swing -= error
	return swing


func _luff_axis(geometry: Dictionary) -> Vector3:
	return (geometry["head"] - geometry["tack"]).normalized()


func _fail(message: String) -> bool:
	last_error = message
	push_error("BoatVisual: " + message)
	return false


func _read_json(path: String) -> Dictionary:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		_fail("cannot open " + path)
		return {}
	var parsed = JSON.parse_string(file.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		_fail("cannot parse " + path)
		return {}
	return parsed


func _vec3(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))


# Reads an OBJ at runtime rather than importing it: the dossier is the single source and the folder
# is .gdignore'd, so the same files the simulation loads are the ones drawn, with no copy to
# regenerate. Returns object name to {"vertices": PackedVector3Array, "indices": PackedInt32Array},
# triangulating polygons as fans. A file with no "o" lines is one object named "".
func _read_obj(path: String) -> Dictionary:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		_fail("cannot open " + path)
		return {}
	var positions := PackedVector3Array()
	var objects := {}
	var current := ""
	var remap := {}
	var vertices := PackedVector3Array()
	var indices := PackedInt32Array()
	var started := false
	while not file.eof_reached():
		var line := file.get_line()
		if line.begins_with("v "):
			var parts := line.split(" ", false)
			positions.append(Vector3(float(parts[1]), float(parts[2]), float(parts[3])))
		elif line.begins_with("o "):
			if started:
				objects[current] = {"vertices": vertices, "indices": indices}
			current = line.substr(2).strip_edges()
			remap = {}
			vertices = PackedVector3Array()
			indices = PackedInt32Array()
			started = true
		elif line.begins_with("f "):
			started = true
			var parts := line.split(" ", false)
			var face := PackedInt32Array()
			for i in range(1, parts.size()):
				var source := int(parts[i].get_slice("/", 0)) - 1
				if not remap.has(source):
					remap[source] = vertices.size()
					vertices.append(positions[source])
				face.append(remap[source])
			for i in range(1, face.size() - 1):
				indices.append_array(PackedInt32Array([face[0], face[i], face[i + 1]]))
	if started:
		objects[current] = {"vertices": vertices, "indices": indices}
	return objects


func _mesh_from_obj(object: Dictionary) -> ArrayMesh:
	return _indexed_mesh(object["vertices"], object["indices"])


# Smooth, area-weighted vertex normals. The input triangles are counter-clockwise seen from outside,
# as in the dossier OBJs; Godot's front faces are clockwise, so the emitted indices swap two
# vertices. Left as they were, every face would be a back face and the renderer would light the
# hull with its normals flipped inward.
func _indexed_mesh(vertices: PackedVector3Array, indices: PackedInt32Array) -> ArrayMesh:
	var normals := PackedVector3Array()
	normals.resize(vertices.size())
	for t in range(0, indices.size(), 3):
		var a := indices[t]
		var b := indices[t + 1]
		var c := indices[t + 2]
		var face_normal := (vertices[b] - vertices[a]).cross(vertices[c] - vertices[a])
		normals[a] += face_normal
		normals[b] += face_normal
		normals[c] += face_normal
	for i in normals.size():
		normals[i] = normals[i].normalized()
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	var clockwise := indices.duplicate()
	for t in range(0, clockwise.size(), 3):
		clockwise[t + 1] = indices[t + 2]
		clockwise[t + 2] = indices[t + 1]
	arrays[Mesh.ARRAY_INDEX] = clockwise
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


# A flat sail in the centreplane of the hull frame as a fan from its centroid over the outline
# tack, head, leech (bowed outward by `roach`), clew, back to the tack along the foot.
func _sail_node(node_name: String, geometry: Dictionary, material: Material) -> MeshInstance3D:
	var tack: Vector3 = geometry["tack"]
	var head: Vector3 = geometry["head"]
	var clew: Vector3 = geometry["clew"]
	var roach: float = geometry["roach"]
	var outline := PackedVector3Array([Vector3.ZERO, head - tack])
	if roach > 0.0:
		var leech := clew - head
		var plane_normal := (head - tack).cross(leech)
		var outward := plane_normal.cross(leech).normalized()
		if outward.dot((head + clew) * 0.5 - tack) < 0.0:
			outward = -outward
		for i in range(1, ROACH_POINTS + 1):
			var t := float(i) / (ROACH_POINTS + 1)
			outline.append(head - tack + leech * t + outward * (roach * sin(PI * t)))
	outline.append(clew - tack)

	var centroid := Vector3.ZERO
	for point in outline:
		centroid += point
	centroid /= outline.size()
	var vertices := outline.duplicate()
	vertices.append(centroid)
	var hub := vertices.size() - 1
	var indices := PackedInt32Array()
	for i in outline.size():
		indices.append_array(PackedInt32Array([hub, i, (i + 1) % outline.size()]))
	return _mesh_node(node_name, _indexed_mesh(vertices, indices), material)


func _mesh_node(node_name: String, mesh: Mesh, material: Material) -> MeshInstance3D:
	var node := MeshInstance3D.new()
	node.name = node_name
	node.mesh = mesh
	node.material_override = material
	return node


func _box_node(node_name: String, size: Vector3, material: Material) -> MeshInstance3D:
	var box := BoxMesh.new()
	box.size = size
	return _mesh_node(node_name, box, material)


func _cylinder_node(node_name: String, from: Vector3, to: Vector3, radius: float,
		material: Material) -> MeshInstance3D:
	var cylinder := CylinderMesh.new()
	cylinder.top_radius = radius
	cylinder.bottom_radius = radius
	cylinder.height = from.distance_to(to)
	var node := _mesh_node(node_name, cylinder, material)
	node.basis = Basis(Quaternion(Vector3.UP, (to - from).normalized()))
	node.position = (from + to) * 0.5
	return node


func _material(colour: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = colour
	material.roughness = 0.6
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	return material


func _sail_material() -> StandardMaterial3D:
	return _material(SAIL_COLOUR)
