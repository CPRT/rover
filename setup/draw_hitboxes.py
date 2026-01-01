#!/usr/bin/env python3

"""
Use to draw simple box/cylinder hitboxes over an STL mesh, then export URDF <collision> blocks.
"""
import argparse
import math
import numpy as np
import trimesh

from PyQt5 import QtWidgets, QtCore
import pyvista as pv
from pyvistaqt import QtInteractor


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    R = rpy_to_R(*rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def fmt(v):
    return " ".join(f"{x:.6f}" for x in v)


class Primitive:
    def __init__(self, prim_type="box"):
        self.type = prim_type  # "box" or "cyl"
        self.xyz = np.array([0.0, 0.0, 0.0], dtype=float)
        self.rpy = np.array([0.0, 0.0, 0.0], dtype=float)

        if prim_type == "box":
            self.size = np.array([0.1, 0.1, 0.1], dtype=float)  # x y z
            self.radius = None
            self.length = None
        else:
            self.radius = 0.05
            self.length = 0.2
            self.size = None

    def urdf_collision(self):
        if self.type == "box":
            return f"""  <collision>
    <origin xyz="{fmt(self.xyz)}" rpy="{fmt(self.rpy)}"/>
    <geometry>
      <box size="{fmt(self.size)}"/>
    </geometry>
  </collision>"""
        else:
            return f"""  <collision>
    <origin xyz="{fmt(self.xyz)}" rpy="{fmt(self.rpy)}"/>
    <geometry>
      <cylinder radius="{self.radius:.6f}" length="{self.length:.6f}"/>
    </geometry>
  </collision>"""

    def build_mesh(self):
        if self.type == "box":
            m = trimesh.creation.box(extents=self.size)
        else:
            m = trimesh.creation.cylinder(
                radius=self.radius, height=self.length, sections=64
            )

        T = make_T(self.xyz, self.rpy)
        m.apply_transform(T)
        return m


class HitboxEditor(QtWidgets.QMainWindow):
    def __init__(self, stl_path, scale=1.0):
        super().__init__()
        self.setWindowTitle("Hitbox Drawer (Boxes/Cylinders) -> URDF Collisions")
        self.resize(1400, 900)

        self.prims = []
        self.actors = []  # pyvista actors per primitive
        self.selected = -1

        # Load STL with trimesh, convert to pyvista
        mesh = trimesh.load_mesh(stl_path, force="mesh")
        if scale != 1.0:
            mesh.apply_scale(scale)
        self.mesh_trimesh = mesh

        pv_mesh = pv.wrap(mesh.as_open3d.triangle_mesh) if False else None  # not used
        # Create PyVista PolyData directly
        verts = mesh.vertices
        faces = np.hstack([np.full((mesh.faces.shape[0], 1), 3), mesh.faces]).astype(
            np.int64
        )
        faces = faces.reshape(-1)
        self.pv_stl = pv.PolyData(verts, faces)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # 3D viewer
        self.plotter = QtInteractor(central)
        layout.addWidget(self.plotter.interactor, stretch=3)

        # Right panel
        panel = QtWidgets.QWidget()
        layout.addWidget(panel, stretch=1)
        pl = QtWidgets.QVBoxLayout(panel)

        # Buttons
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_add_box = QtWidgets.QPushButton("New Box")
        self.btn_add_cyl = QtWidgets.QPushButton("New Cylinder")
        self.btn_del = QtWidgets.QPushButton("Delete")
        btn_row.addWidget(self.btn_add_box)
        btn_row.addWidget(self.btn_add_cyl)
        btn_row.addWidget(self.btn_del)
        pl.addLayout(btn_row)

        # Primitive list
        self.listw = QtWidgets.QListWidget()
        pl.addWidget(self.listw)

        # Form fields
        form = QtWidgets.QFormLayout()
        pl.addLayout(form)

        self.x = QtWidgets.QDoubleSpinBox()
        self.x.setRange(-1000, 1000)
        self.x.setDecimals(6)
        self.x.setSingleStep(0.01)
        self.y = QtWidgets.QDoubleSpinBox()
        self.y.setRange(-1000, 1000)
        self.y.setDecimals(6)
        self.y.setSingleStep(0.01)
        self.z = QtWidgets.QDoubleSpinBox()
        self.z.setRange(-1000, 1000)
        self.z.setDecimals(6)
        self.z.setSingleStep(0.01)

        self.roll = QtWidgets.QDoubleSpinBox()
        self.roll.setRange(-math.pi, math.pi)
        self.roll.setDecimals(6)
        self.roll.setSingleStep(0.01)
        self.pitch = QtWidgets.QDoubleSpinBox()
        self.pitch.setRange(-math.pi, math.pi)
        self.pitch.setDecimals(6)
        self.pitch.setSingleStep(0.01)
        self.yaw = QtWidgets.QDoubleSpinBox()
        self.yaw.setRange(-math.pi, math.pi)
        self.yaw.setDecimals(6)
        self.yaw.setSingleStep(0.01)

        self.sx = QtWidgets.QDoubleSpinBox()
        self.sx.setRange(0.0001, 1000)
        self.sx.setDecimals(6)
        self.sx.setSingleStep(0.01)
        self.sy = QtWidgets.QDoubleSpinBox()
        self.sy.setRange(0.0001, 1000)
        self.sy.setDecimals(6)
        self.sy.setSingleStep(0.01)
        self.sz = QtWidgets.QDoubleSpinBox()
        self.sz.setRange(0.0001, 1000)
        self.sz.setDecimals(6)
        self.sz.setSingleStep(0.01)

        self.radius = QtWidgets.QDoubleSpinBox()
        self.radius.setRange(0.0001, 1000)
        self.radius.setDecimals(6)
        self.radius.setSingleStep(0.01)
        self.length = QtWidgets.QDoubleSpinBox()
        self.length.setRange(0.0001, 1000)
        self.length.setDecimals(6)
        self.length.setSingleStep(0.01)

        form.addRow("x", self.x)
        form.addRow("y", self.y)
        form.addRow("z", self.z)
        form.addRow("roll", self.roll)
        form.addRow("pitch", self.pitch)
        form.addRow("yaw", self.yaw)
        form.addRow("box sx", self.sx)
        form.addRow("box sy", self.sy)
        form.addRow("box sz", self.sz)
        form.addRow("cyl radius", self.radius)
        form.addRow("cyl length", self.length)

        # Export
        self.btn_export = QtWidgets.QPushButton("Copy URDF collisions to clipboard")
        pl.addWidget(self.btn_export)

        # Viewer setup
        self.plotter.add_mesh(self.pv_stl, opacity=0.25)
        self.plotter.add_axes()
        self.plotter.reset_camera()

        # Signals
        self.btn_add_box.clicked.connect(self.add_box)
        self.btn_add_cyl.clicked.connect(self.add_cyl)
        self.btn_del.clicked.connect(self.delete_selected)
        self.listw.currentRowChanged.connect(self.select_index)

        for w in [
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
            self.sx,
            self.sy,
            self.sz,
            self.radius,
            self.length,
        ]:
            w.valueChanged.connect(self.on_fields_changed)

        self.btn_export.clicked.connect(self.export_urdf)

        self.update_fields_enabled(False)

    def update_fields_enabled(self, enabled):
        for w in [
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
            self.sx,
            self.sy,
            self.sz,
            self.radius,
            self.length,
        ]:
            w.setEnabled(enabled)

    def add_box(self):
        p = Primitive("box")
        # initialize roughly at mesh center
        p.xyz = self.mesh_trimesh.bounding_box.centroid.copy()
        # size roughly from mesh extents / 4
        ext = self.mesh_trimesh.bounding_box.extents
        p.size = np.maximum(ext / 4.0, 0.02)
        self.prims.append(p)
        self.listw.addItem(f"box_{len(self.prims)-1}")
        self.spawn_actor(len(self.prims) - 1)
        self.listw.setCurrentRow(len(self.prims) - 1)

    def add_cyl(self):
        p = Primitive("cyl")
        p.xyz = self.mesh_trimesh.bounding_box.centroid.copy()
        ext = self.mesh_trimesh.bounding_box.extents
        p.length = float(max(ext) / 3.0)
        p.radius = float(max(min(ext) / 6.0, 0.01))
        self.prims.append(p)
        self.listw.addItem(f"cyl_{len(self.prims)-1}")
        self.spawn_actor(len(self.prims) - 1)
        self.listw.setCurrentRow(len(self.prims) - 1)

    def spawn_actor(self, idx):
        prim = self.prims[idx]
        tm = prim.build_mesh()

        verts = tm.vertices
        faces = (
            np.hstack([np.full((tm.faces.shape[0], 1), 3), tm.faces])
            .astype(np.int64)
            .reshape(-1)
        )
        pd = pv.PolyData(verts, faces)

        actor = self.plotter.add_mesh(pd, opacity=0.35)
        self.actors.append((pd, actor))
        self.plotter.render()

    def update_actor(self, idx):
        prim = self.prims[idx]
        tm = prim.build_mesh()

        pd, actor = self.actors[idx]
        # Update points and faces (faces constant for box/cyl? safer to rebuild)
        verts = tm.vertices
        faces = (
            np.hstack([np.full((tm.faces.shape[0], 1), 3), tm.faces])
            .astype(np.int64)
            .reshape(-1)
        )
        new_pd = pv.PolyData(verts, faces)

        # Remove old actor and re-add (simple + reliable)
        self.plotter.remove_actor(actor)
        new_actor = self.plotter.add_mesh(new_pd, opacity=0.35)
        self.actors[idx] = (new_pd, new_actor)
        self.plotter.render()

    def delete_selected(self):
        if self.selected < 0 or self.selected >= len(self.prims):
            return
        _, actor = self.actors[self.selected]
        self.plotter.remove_actor(actor)

        del self.prims[self.selected]
        del self.actors[self.selected]
        self.listw.takeItem(self.selected)

        # Re-label list
        for i in range(self.listw.count()):
            t = self.listw.item(i).text()
            kind = "box" if "box" in t else "cyl"
            self.listw.item(i).setText(f"{kind}_{i}")

        self.selected = -1
        self.update_fields_enabled(False)
        self.plotter.render()

    def select_index(self, idx):
        self.selected = idx
        if idx < 0 or idx >= len(self.prims):
            self.update_fields_enabled(False)
            return

        self.update_fields_enabled(True)
        prim = self.prims[idx]

        # Block signals while populating fields
        ws = [
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
            self.sx,
            self.sy,
            self.sz,
            self.radius,
            self.length,
        ]
        for w in ws:
            w.blockSignals(True)

        self.x.setValue(float(prim.xyz[0]))
        self.y.setValue(float(prim.xyz[1]))
        self.z.setValue(float(prim.xyz[2]))
        self.roll.setValue(float(prim.rpy[0]))
        self.pitch.setValue(float(prim.rpy[1]))
        self.yaw.setValue(float(prim.rpy[2]))

        if prim.type == "box":
            self.sx.setValue(float(prim.size[0]))
            self.sy.setValue(float(prim.size[1]))
            self.sz.setValue(float(prim.size[2]))
        else:
            self.radius.setValue(float(prim.radius))
            self.length.setValue(float(prim.length))

        # Enable/disable size fields based on type
        is_box = prim.type == "box"
        self.sx.setEnabled(is_box)
        self.sy.setEnabled(is_box)
        self.sz.setEnabled(is_box)
        self.radius.setEnabled(not is_box)
        self.length.setEnabled(not is_box)

        for w in ws:
            w.blockSignals(False)

    def on_fields_changed(self):
        if self.selected < 0 or self.selected >= len(self.prims):
            return
        prim = self.prims[self.selected]

        prim.xyz = np.array(
            [self.x.value(), self.y.value(), self.z.value()], dtype=float
        )
        prim.rpy = np.array(
            [self.roll.value(), self.pitch.value(), self.yaw.value()], dtype=float
        )

        if prim.type == "box":
            prim.size = np.array(
                [self.sx.value(), self.sy.value(), self.sz.value()], dtype=float
            )
        else:
            prim.radius = float(self.radius.value())
            prim.length = float(self.length.value())

        self.update_actor(self.selected)

    def export_urdf(self):
        blocks = [p.urdf_collision() for p in self.prims]
        out = "\n".join(blocks)

        cb = QtWidgets.QApplication.clipboard()
        cb.setText(out)

        QtWidgets.QMessageBox.information(
            self, "Copied", "URDF <collision> blocks copied to clipboard."
        )


def main():
    ap = argparse.ArgumentParser(
        description="Interactive collision primitive drawer for an STL."
    )
    ap.add_argument("stl", help="Path to STL file")
    ap.add_argument(
        "--scale", type=float, default=1.0, help="Scale factor (e.g. 0.001 for mm->m)"
    )
    args = ap.parse_args()

    app = QtWidgets.QApplication([])
    w = HitboxEditor(args.stl, scale=args.scale)
    w.show()
    app.exec_()


if __name__ == "__main__":
    main()
