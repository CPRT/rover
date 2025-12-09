# GridMap ↔ CuPy Memory Layout Alignment Plan

## Context
- **GridMap / Eigen**: column-major (Fortran) by default.
- **CuPy / NumPy**: row-major (C) by default.
- Current code works but relies on implicit defaults; we should make conversions explicit to avoid silent axis/stride bugs.

## Goals
- Keep internal CuPy tensors row-major (C) for existing ops and axis semantics.
- Be explicit at boundaries (GridMap messages / bag I/O): convert to column-major when publishing, and normalize to row-major when consuming.
- Document intent in code and add minimal tests to guard behavior.

## Changes to Implement
1) **Document intent**
   - In `elevation_mapping.py` near GridMap↔array helpers, add a brief note: internal arrays are C-order; GridMap expects F-order; conversions are explicit.

2) **Inbound (GridMap → NumPy/CuPy)**
   - In `_grid_map_to_numpy` (uses `_extract_layout_shape`):
     - Detect layout orientation via `layout.dim[0].label` (default to column-major if absent).
     - For column-major inputs: `np.array(..., order='F').reshape((rows, cols), order='F')`, then `np.ascontiguousarray` for internal row-major use.
     - For row-major inputs: keep `order='C'`.
   - Encapsulate this in a small helper (e.g., `_grid_msg_to_array(msg, expected_order='F')`) and reuse in set_full_map/bag reads.

3) **Outbound (NumPy/CuPy → GridMap)**
   - In `_numpy_to_multiarray`:
     - Ensure column-major with `np.asfortranarray(data)` before flattening.
     - Set `layout.dim[0].label` to indicate column-major (e.g., "column_index" consistent with Eigen).
   - If internal is row-major, transpose if needed before `asfortranarray` so axes stay correct.

4) **Internal exports/imports**
   - In `export_layers`, `set_full_map`, and bag I/O helpers, route through the same normalize/fortranize helpers to keep consistency everywhere.

5) **Tests**
   - Add a unit test around the conversion helpers:
     - Build a small grid with distinct row/col values, serialize to GridMap msg, deserialize, and assert axes/values preserved.
     - Check `layout.dim[0].label` is set and respected on ingest.

6) **Runtime defaults**
   - No change to `shift_map_xy` or other core ops; all changes are at message/bag boundaries.

## Notes
- This keeps existing behavior while making layout assumptions explicit and guarded by tests.
