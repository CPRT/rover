# Index / Layout Refactor Plan

Goal: make all internal CuPy/NumPy operations use a **row‑major** convention (array shape `(rows, cols)` with `rows = Y`, `cols = X`), while still interoperating cleanly with the **column‑major** GridMap conventions on the ROS side.

This document describes the target invariants and a step‑by‑step plan to move the `dev/ros2/update_map_rebase` branch to that design without re‑introducing axis/flip bugs.

---

## 1. Target Invariants

We want the following to be true after the refactor:

- **Internal representation (CuPy/NumPy)**
  - All 2D map buffers used by `ElevationMap` are treated as **row‑major**: `array[row, col]` where:
    - `row` indexes **Y** (northing).
    - `col` indexes **X** (easting).
  - The 3D elevation map tensor has shape `(layers, rows, cols)` and is always interpreted with `(rows = Y, cols = X)`.
  - `shift_map_xy` is the only place where we translate `[dx, dy]` world shifts into roll indices for `(rows, cols)` (this is already enforced by the axis‑swap tests).

- **Geometry and indexing**
  - `GridGeometry.shape` returns `(rows, cols)` computed as `(length_y / res, length_x / res)`.
  - Anywhere we compute pixel indices from world coordinates, we consistently use:
    - `row = f(y)`, `col = f(x)`.
  - `_compute_overlap_indices` returns slices `(rows_slice, cols_slice)` and respects the Y→rows, X→cols convention.
  - `_map_extent_from_slices` / `_map_extent_from_mask` invert that mapping correctly back to metric X/Y extents.

- **ROS GridMap messages**
  - Internally we always consume/produce **row‑major 2D arrays**.
  - At the ROS boundary we can:
    - **Read** both:
      - row‑major messages: `dim[0] = "row_index", dim[1] = "column_index"`, C‑order layout.
      - legacy GridMap‑style column‑major messages: `dim[0] = "column_index", dim[1] = "row_index"`, column‑major layout.
    - **Write** GridMap messages in a *single, documented* convention:
      - Preferred: **column‑major external layout** (grid_map compatible) with `dim[0] = "column_index"`, `dim[1] = "row_index"`, and `data[col * rows + row]` encoding.

---

## 2. Current State (dev/ros2/update_map_rebase)

Short summary of the relevant pieces on the rebase branch:

- `ElevationMap`:
  - Internal tensors are already treated as `(layers, rows, cols)`, with Y mapped to rows and X to cols.
  - `GridGeometry.shape` returns `(rows, cols)` and is used to validate incoming patches and full‑map restores.
  - `_compute_overlap_indices` and `_map_extent_from_*` operate in row/col slices and compute metric X/Y correctly.

- Node publisher (`ElevationMappingNode.publish_map`):
  - Fills a NumPy buffer `self._map_data` with shape `(rows, cols)` via `get_map_with_name_ref`.
  - Manually builds `Float32MultiArray`:
    - `dim[0] = ("column_index", size=cols, stride=rows*cols)`
    - `dim[1] = ("row_index", size=rows, stride=rows)`
    - `data = array.flatten(order="C").tolist()`
  - This is effectively a **column‑major logical layout**, but the reshape semantics are not explicitly tied to Fortran order.

- Save/load pipeline (`handle_save_map`, `handle_load_map`):
  - Uses `_build_grid_map_message` → `_numpy_to_multiarray` for writing.
  - Uses `_grid_map_to_numpy` → `_float32_multiarray_to_numpy` for reading.
  - On the rebase branch, `_numpy_to_multiarray` was changed toward a **row‑major** layout (row_index first).
  - `_float32_multiarray_to_numpy` tries to accept both row‑major and column‑major label orderings and normalize them to our internal row‑major arrays.

- Masked replace:
  - `handle_masked_replace` uses `_grid_map_to_numpy` to interpret the incoming `GridMap` (including the mask layer) into `(rows, cols)` NumPy arrays and passes those into `ElevationMap.apply_masked_replace`.
  - `scripts/masked_replace_tool.py` currently has its own `_numpy_to_multiarray` that does not necessarily match the node’s helper exactly.

Result: we already **normalize incoming messages** into row‑major arrays, but our **writers are inconsistent** (publisher vs save_map, node vs CLI), and column‑major vs row‑major semantics are mixed at the edges.

---

## 3. Desired External Semantics

To keep the internal code simple and still play nicely with GridMap tooling:

- **Internal (CuPy/NumPy):** always row‑major, `(rows, cols) = (Y, X)`.
- **External GridMap messages (topics + bags):**
  - Default to **GridMap‑style column‑major encoding**, because:
    - That is what C++ `grid_map` and downstream tools expect.
    - It keeps RViz and other consumers compatible.
  - But we still accept both:
    - Row‑major (`row_index`, `column_index`).
    - Column‑major (`column_index`, `row_index`).

Concretely, for a row‑major internal array `M` with shape `(rows, cols)`:

- **Publish/save (column‑major GridMap)**:
  - `dim[0].label = "column_index"`, `size = cols`, `stride = rows * cols`.
  - `dim[1].label = "row_index"`, `size = rows`, `stride = rows`.
  - `data[k]` encodes `M[row, col]` with:
    - `flat_idx = col * rows + row`.
  - Implementation‑wise this is equivalent to:
    - `data = M.flatten(order="F")`  (Fortran‑order flatten).

- **Read (normalize to row‑major)**:
  - If `dim[0] = "row_index"`, `dim[1] = "column_index"`:
    - Treat data as row‑major: `M = data_np.reshape((rows, cols), order="C")`.
  - If `dim[0] = "column_index"`, `dim[1] = "row_index"`:
    - Treat data as column‑major GridMap: `M = data_np.reshape((rows, cols), order="F")`.
  - Either way, `M[row, col]` matches the semantic cell at (Y=row, X=col).

This keeps **internal invariants** clean and makes the **GridMap boundary** a pure format conversion layer.

---

## 4. Refactor Plan

### 4.1. Centralize conversion helpers in the node

**Goal:** one code path that knows how to go between internal `(rows, cols)` arrays and `Float32MultiArray` for GridMap messages.

Steps:

1. In `elevation_mapping_cupy/scripts/elevation_mapping_node.py`, design a pair of helpers:

   - `_encode_layer_to_multiarray(array: np.ndarray, layout: str = "gridmap_column") -> Float32MultiArray`
     - `array` is `(rows, cols)` row‑major.
     - `layout` options:
       - `"gridmap_column"` (default): emit GridMap‑style column‑major.
       - `"row_major"` (optional, for debugging/regression).
     - For `"gridmap_column"`:
       - `rows, cols = array.shape`.
       - Set layout dims:
         - `dim[0] = (label="column_index", size=cols, stride=rows * cols)`.
         - `dim[1] = (label="row_index", size=rows, stride=rows)`.
       - `msg.data = array.flatten(order="F").tolist()`.
     - For `"row_major"`:
       - `dim[0] = (label="row_index", size=rows, stride=rows * cols)`.
       - `dim[1] = (label="column_index", size=cols, stride=cols)`.
       - `msg.data = array.flatten(order="C").tolist()`.

   - `_decode_multiarray_to_rows_cols(name: str, msg: Float32MultiArray) -> np.ndarray`
     - Inspect `msg.layout.dim[0].label`, `msg.layout.dim[1].label`.
     - Cases:
       - `"row_index"`, `"column_index"`:
         - `rows = dim[0].size or inferred`, `cols = dim[1].size or inferred`.
         - `array = data_np.reshape((rows, cols), order="C")`.
       - `"column_index"`, `"row_index"`:
         - `cols = dim[0].size or inferred`, `rows = dim[1].size or inferred`.
         - `array = data_np.reshape((rows, cols), order="F")`.
       - Fallback (no labels):
         - Infer square or `cols, rows` from first/second dim and use C‑order.
     - Always return a `(rows, cols)` **row‑major** array that matches our internal convention.

2. Replace:
   - Existing `_numpy_to_multiarray` with `_encode_layer_to_multiarray` (or keep name but extend as above).
   - Existing `_float32_multiarray_to_numpy` with `_decode_multiarray_to_rows_cols`.

3. Update all call sites in the node:
   - `publish_map`:
     - Replace manual `Float32MultiArray` construction with:
       - `gm.data.append(self._encode_layer_to_multiarray(map_data_for_gridmap, layout="gridmap_column"))`.
   - `_build_grid_map_message` (used by `handle_save_map`):
     - Use `_encode_layer_to_multiarray(..., layout="gridmap_column")`.
   - `_grid_map_to_numpy`:
     - Use `_decode_multiarray_to_rows_cols` per layer name.

Result: topics and saved GridMaps both use the same external encoding, and we have a single, testable conversion layer.

### 4.2. Keep internal row‑major semantics untouched

We must ensure the refactor does not change internal semantics:

- Leave `ElevationMap` and `GridGeometry` as‑is:
  - `GridGeometry.shape` = `(rows, cols)`.
  - `_compute_overlap_indices` uses `(rows_slice, cols_slice)` and bounds in metric X/Y.
  - `_map_extent_from_slices` and `_map_extent_from_mask` convert slices back to X/Y using the same row/col ↔ Y/X mapping.
  - `shift_map_xy` keeps the corrected `[dx, dy] → [row_shift=dy, col_shift=dx]` mapping (from 85591c24…).

The only changes happen in the *Node* and CLI, not in the core CuPy code.

### 4.3. Unify CLI tool encoding (`masked_replace_tool.py`)

The CLI tool should emit the **same layout** as the node expects by default:

1. In `scripts/masked_replace_tool.py`, change `_numpy_to_multiarray` to mirror the node’s `"gridmap_column"` encoding:
   - `rows, cols = array.shape`.
   - `dim[0] = ("column_index", size=cols, stride=rows * cols)`.
   - `dim[1] = ("row_index", size=rows, stride=rows)`.
   - `data = array.flatten(order="F").tolist()`.

2. Optionally:
   - Add a `--layout` flag (e.g. `gridmap_column` vs `row_major`) if we want to be able to generate both, but this is not required now; simpler is better.

Since the node’s decoder will accept both layouts, this is mostly about **avoiding surprises** and making it easy to reason about tests.

### 4.4. Tests for conversion correctness

Add focused tests under `elevation_mapping_cupy/elevation_mapping_cupy/tests/`:

1. **Round‑trip conversion tests (no ROS)**:
   - Create a small synthetic array `M` with distinct values (e.g. `M[row, col] = row * 10 + col`).
   - Encode with `"gridmap_column"` layout:
     - `msg = _encode_layer_to_multiarray(M, layout="gridmap_column")`.
   - Decode:
     - `M_dec = _decode_multiarray_to_rows_cols("elevation", msg)`.
   - Assert `np.array_equal(M, M_dec)`.

   Repeat for `"row_major"` layout to validate both code paths.

2. **GridMap index semantics test**:
   - Manually construct a `Float32MultiArray` emulating standard C++ `grid_map`:
     - `dim[0] = ("column_index", size=cols, stride=rows * cols)`.
     - `dim[1] = ("row_index", size=rows, stride=rows)`.
     - `data[k] = col * rows + row`.
   - Use `_decode_multiarray_to_rows_cols` and assert that `array[row, col] == row * 10 + col` (or similar) to confirm we are reading column‑major correctly.

3. **Service / mask placement tests** (extend `test_map_services.py`):
   - Construct a map and a `GridGeometry` as in `test_apply_masked_replace_updates_layer`.
   - Build a synthetic `GridMap` message where:
     - The mask is a small sub‑window (e.g. square patch at a known region).
     - Data is encoded using `"gridmap_column"` layout for the mask and data layers.
   - Call `apply_masked_replace` (directly on `ElevationMap` or via the node) and verify:
     - Only the expected cells in the internal `(rows, cols)` buffer were changed.
     - The metric extents computed by `_map_extent_from_mask` match the intended patch location.

4. **Save/load identity test**:
   - Programmatically:
     - Fill `ElevationMap` layers with a known pattern.
     - Use `handle_save_map` to write bags to a temporary directory.
     - Use `handle_load_map` to reload.
     - Compare the internal layers (`export_layers(...)`) before and after; they should match bit‑for‑bit.

These tests ensure the conversion layer is correct and that masks/patches land in the right cells.

### 4.5. Documentation updates

1. Add a short “Indexing & Layout” section to the developer docs (or README) summarizing:
   - Internal convention: `(rows, cols) = (Y, X)` row‑major.
   - External GridMap convention: column‑major (`column_index` first) for topics and bags.
   - The existence of a single conversion layer in `ElevationMappingNode` that handles both directions and both label orders.

2. Keep this `index_refacto_plan.md` around as a reference while we implement; once refactor is complete and stable, we can:
   - Either trim it down to a short design note.
   - Or move the key parts into the main docs and delete this file.

---

## 5. Implementation Order / Risk Control

To minimize risk of regressions, implement in this order:

1. Introduce `_encode_layer_to_multiarray` and `_decode_multiarray_to_rows_cols` in the node, **without** changing existing call sites.
2. Add unit tests for these helpers (round‑trip, GridMap semantics).
3. Switch `publish_map` to use the new encoder; re‑run tests.
4. Switch `_build_grid_map_message` to use the new encoder; re‑run tests, especially `test_map_services.py`.
5. Switch `_grid_map_to_numpy` to use the new decoder; re‑run tests.
6. Update `masked_replace_tool.py` encoding; re‑run service tests that involve masked_replace.
7. Finally, run the full pytest suite and any launch/integration tests we have for TF→GridMap and movement (axis‑swap regression).

At each step, verify:

- Axis‑swap tests for `shift_map_xy` still pass.
- Masked replace integration still writes to the correct cells.
- Save→load→compare is an identity operation on internal layers.

