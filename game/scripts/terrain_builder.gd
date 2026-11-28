class_name TerrainBuilder
extends Node3D

## Builds the land and seabed mesh from a heightmap PNG and its meta.json (written by
## tools/scene/make_bay.py) when the node enters the tree. Core world (x east, y north, z up) maps
## to Godot as (x, z, -y).

const PNG_SIGNATURE_HEX := "89504e470d0a1a0a"

const SAND := Color(0.82, 0.74, 0.55)
const GRASS := Color(0.33, 0.50, 0.22)
const ROCK := Color(0.47, 0.44, 0.40)
const SHALLOW := Color(0.62, 0.66, 0.48)
const DEEP := Color(0.10, 0.20, 0.28)

@export_dir var scene_dir := "res://data/scenes/bay"
## Mesh vertex spacing; rounded to a whole number of heightmap pixels.
@export var grid_step_m := 8.0

var meta := {}
var width_px := 0
var height_px := 0
var pixel_size_m := 0.0
var origin_x_m := 0.0
var origin_y_m := 0.0
var height_scale_m := 0.0
var height_offset_m := 0.0
var grid_columns := 0
var grid_rows := 0
var build_msec := 0
var last_error := ""
var mesh_instance: MeshInstance3D

var _samples := PackedByteArray()


func _ready() -> void:
	if mesh_instance == null and not build():
		push_error("TerrainBuilder: " + last_error)


func is_built() -> bool:
	return mesh_instance != null


func build() -> bool:
	var started := Time.get_ticks_msec()
	if not _load_meta() or not _load_heightmap():
		return false
	var mesh := _build_mesh()
	mesh_instance = MeshInstance3D.new()
	mesh_instance.name = "TerrainMesh"
	mesh_instance.mesh = mesh
	add_child(mesh_instance)
	build_msec = Time.get_ticks_msec() - started
	return true


## Bilinear height in metres at core world (x, y), as meta.json's sampling block describes.
func height_at_world(x_m: float, y_m: float) -> float:
	var col := clampf((x_m - origin_x_m) / pixel_size_m, 0.0, width_px - 1.0)
	var row := clampf((origin_y_m - y_m) / pixel_size_m, 0.0, height_px - 1.0)
	var c0 := mini(floori(col), width_px - 2)
	var r0 := mini(floori(row), height_px - 2)
	var tc := col - c0
	var tr := row - r0
	var top := lerpf(_height_px(c0, r0), _height_px(c0 + 1, r0), tc)
	var bottom := lerpf(_height_px(c0, r0 + 1), _height_px(c0 + 1, r0 + 1), tc)
	return lerpf(top, bottom, tr)


func height_at_godot(point: Vector3) -> float:
	return height_at_world(point.x, -point.z)


func start_position_godot() -> Vector3:
	var start: Dictionary = meta["start"]
	return Vector3(float(start["x_m"]), 0.0, -float(start["y_m"]))


func _fail(message: String) -> bool:
	last_error = message
	return false


func _load_meta() -> bool:
	var path := scene_dir.path_join("meta.json")
	var text := FileAccess.get_file_as_string(path)
	if text.is_empty():
		return _fail("cannot read " + path)
	var parsed = JSON.parse_string(text)
	if not parsed is Dictionary:
		return _fail(path + " is not a JSON object")
	meta = parsed
	pixel_size_m = float(meta["pixel_size_m"])
	origin_x_m = float(meta["origin"]["x_m"])
	origin_y_m = float(meta["origin"]["y_m"])
	height_scale_m = float(meta["height_encoding"]["scale_m"])
	height_offset_m = float(meta["height_encoding"]["offset_m"])
	if pixel_size_m <= 0.0:
		return _fail(path + ": pixel_size_m must be positive")
	return true


## A minimal reader for the PNGs make_bay.py writes. Godot's own PNG loader reduces 16-bit samples
## to 8 bits, which would quantise heights to about 0.6 m.
func _load_heightmap() -> bool:
	var path := scene_dir.path_join(str(meta["heightmap"]["file"]))
	var bytes := FileAccess.get_file_as_bytes(path)
	if bytes.size() < 8 or bytes.slice(0, 8).hex_encode() != PNG_SIGNATURE_HEX:
		return _fail(path + " is not a PNG")
	var idat := PackedByteArray()
	var at := 8
	while at + 8 <= bytes.size():
		var length := _be32(bytes, at)
		var kind := bytes.slice(at + 4, at + 8).get_string_from_ascii()
		var body := bytes.slice(at + 8, at + 8 + length)
		if kind == "IHDR":
			width_px = _be32(body, 0)
			height_px = _be32(body, 4)
			if body[8] != 16 or body[9] != 0 or body[12] != 0:
				return _fail(path + ": want 16-bit greyscale, not interlaced")
		elif kind == "IDAT":
			idat.append_array(body)
		elif kind == "IEND":
			break
		at += 12 + length
	if width_px < 2 or height_px < 2:
		return _fail(path + ": missing or too small IHDR")
	var stride := 1 + width_px * 2
	_samples = idat.decompress(stride * height_px, FileAccess.COMPRESSION_DEFLATE)
	if _samples.size() != stride * height_px:
		return _fail(path + ": image data is truncated")
	for row in height_px:
		if _samples[row * stride] != 0:
			return _fail(path + ": row %d uses a PNG filter; only filter type 0 is supported" % row)
	return true


func _be32(bytes: PackedByteArray, at: int) -> int:
	return (bytes[at] << 24) | (bytes[at + 1] << 16) | (bytes[at + 2] << 8) | bytes[at + 3]


func _height_px(col: int, row: int) -> float:
	var at := row * (1 + width_px * 2) + 1 + col * 2
	return float((_samples[at] << 8) | _samples[at + 1]) * height_scale_m + height_offset_m


func _colour(height_m: float, slope: float) -> Color:
	if height_m < 0.0:
		return SHALLOW.lerp(DEEP, clampf(-height_m / 30.0, 0.0, 1.0))
	if height_m < 3.0:
		return SAND
	var colour := GRASS.lerp(ROCK, clampf((height_m - 25.0) / 30.0, 0.0, 1.0))
	return colour.lerp(ROCK, clampf((slope - 0.35) / 0.3, 0.0, 1.0))


func _build_mesh() -> ArrayMesh:
	var step_px := maxi(roundi(grid_step_m / pixel_size_m), 1)
	var step_m := step_px * pixel_size_m
	grid_columns = floori(float(width_px - 1) / step_px) + 1
	grid_rows = floori(float(height_px - 1) / step_px) + 1
	var count := grid_columns * grid_rows

	var heights := PackedFloat32Array()
	heights.resize(count)
	var i := 0
	for r in grid_rows:
		for c in grid_columns:
			heights[i] = _height_px(c * step_px, r * step_px)
			i += 1

	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var colours := PackedColorArray()
	vertices.resize(count)
	normals.resize(count)
	colours.resize(count)
	i = 0
	for r in grid_rows:
		var north := maxi(r - 1, 0) * grid_columns
		var south := mini(r + 1, grid_rows - 1) * grid_columns
		var row_span := (mini(r + 1, grid_rows - 1) - maxi(r - 1, 0)) * step_m
		var godot_z := -(origin_y_m - r * step_m)
		for c in grid_columns:
			var h := heights[i]
			var west := maxi(c - 1, 0)
			var east := mini(c + 1, grid_columns - 1)
			var dh_dx := (heights[r * grid_columns + east] - heights[r * grid_columns + west]) / ((east - west) * step_m)
			var dh_dy := (heights[north + c] - heights[south + c]) / row_span
			vertices[i] = Vector3(origin_x_m + c * step_m, h, godot_z)
			normals[i] = Vector3(-dh_dx, 1.0, dh_dy).normalized()
			colours[i] = _colour(h, sqrt(dh_dx * dh_dx + dh_dy * dh_dy))
			i += 1

	# Godot treats clockwise triangles as front faces; seen from above with north up, each of
	# these runs south-west, north-west, then east.
	var indices := PackedInt32Array()
	indices.resize((grid_columns - 1) * (grid_rows - 1) * 6)
	i = 0
	for r in grid_rows - 1:
		for c in grid_columns - 1:
			var nw := r * grid_columns + c
			var sw := nw + grid_columns
			indices[i] = sw
			indices[i + 1] = nw
			indices[i + 2] = nw + 1
			indices[i + 3] = sw
			indices[i + 4] = nw + 1
			indices[i + 5] = sw + 1
			i += 6

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_COLOR] = colours
	arrays[Mesh.ARRAY_INDEX] = indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)

	var material := StandardMaterial3D.new()
	material.vertex_color_use_as_albedo = true
	material.roughness = 0.95
	mesh.surface_set_material(0, material)
	return mesh
