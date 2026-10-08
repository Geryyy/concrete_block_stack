"""
Line-art render inside Blender: flat fills + Freestyle outlines, transparent background.

Run by `Scene.save` as `blender -b --factory-startup --python blender_render.py -- job.json`.
Blender's Python only: bpy, mathutils, json. The job carries meshes, poses and the
VTK camera, so the overlay projection and this render agree pixel for pixel.
"""

import json
import sys

import bpy
from mathutils import Matrix, Vector

job = json.load(open(sys.argv[sys.argv.index("--") + 1]))
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

meshes = {}  # STL path -> mesh datablock, imported once, shared by every copy
materials = {}


def material(hex_color: str):
    if hex_color not in materials:
        m = bpy.data.materials.new(hex_color)
        m.use_nodes = True
        nodes = m.node_tree.nodes
        nodes.clear()
        rgb = [int(hex_color[i : i + 2], 16) / 255 for i in (1, 3, 5)]
        emit = nodes.new("ShaderNodeEmission")
        emit.inputs["Color"].default_value = (
            *[c**2.2 for c in rgb],
            1,
        )  # sRGB -> linear
        out = nodes.new("ShaderNodeOutputMaterial")
        m.node_tree.links.new(emit.outputs[0], out.inputs[0])
        materials[hex_color] = m
    return materials[hex_color]


collections = {}
for style in ("solid", "ghost"):
    c = bpy.data.collections.new(style)
    scene.collection.children.link(c)
    collections[style] = c

for body in job["bodies"]:
    path = body["mesh"]
    if path not in meshes:
        bpy.ops.wm.stl_import(filepath=path)
        obj = bpy.context.selected_objects[0]
        meshes[path] = obj.data
        bpy.data.objects.remove(obj)
    obj = bpy.data.objects.new("body", meshes[path])
    obj.matrix_world = Matrix(body["matrix"])
    collections[body["style"]].objects.link(obj)
    if body["style"] == "ghost":
        obj.visible_camera = False  # lines only; Freestyle still sees it
        obj.hide_render = False
    obj.data.materials.clear()
    obj.data.materials.append(material(body["fill"]))

cam = job["camera"]
cam_data = bpy.data.cameras.new("cam")
cam_data.sensor_fit = "VERTICAL"
if cam["ortho"]:
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = (
        2 * cam["parallel_scale"] * max(1.0, job["width"] / job["height"])
    )
else:
    cam_data.angle_y = cam["fov"] * 3.141592653589793 / 180
cam_data.clip_start, cam_data.clip_end = 0.05, 500
cam_obj = bpy.data.objects.new("cam", cam_data)
scene.collection.objects.link(cam_obj)
pos, focal, up = Vector(cam["position"]), Vector(cam["focal"]), Vector(cam["up"])
back = (pos - focal).normalized()
right = up.cross(back).normalized()
cam_obj.matrix_world = Matrix(
    [[*right, 0], [*back.cross(right), 0], [*back, 0], [0, 0, 0, 1]]
).transposed() @ Matrix.Identity(4)
cam_obj.location = pos
scene.camera = cam_obj

r = scene.render
r.engine = "CYCLES"
scene.cycles.samples = 4
scene.cycles.use_denoising = False
r.resolution_x, r.resolution_y = job["width"], job["height"]
r.film_transparent = True
r.filepath = job["out"]
r.image_settings.color_mode = "RGBA"
scene.view_settings.view_transform = "Standard"
r.use_freestyle = True
r.line_thickness_mode = "ABSOLUTE"
fs = bpy.context.view_layer.freestyle_settings
fs.crease_angle = job["crease_angle"] * 3.141592653589793 / 180
fs.linesets.remove(fs.linesets[0]) if fs.linesets else None
for style, (color, width) in job["lines"].items():
    ls = fs.linesets.new(style)
    ls.select_by_collection = True
    ls.collection = collections[style]
    ls.select_by_visibility = True
    ls.visibility = "VISIBLE"
    ls.select_silhouette = ls.select_border = ls.select_crease = True
    ls.select_external_contour = True
    rgb = [int(color[i : i + 2], 16) / 255 for i in (1, 3, 5)]
    ls.linestyle.color = [c**2.2 for c in rgb]
    ls.linestyle.thickness = width
    ls.linestyle.caps = "ROUND"
bpy.ops.render.render(write_still=True)
