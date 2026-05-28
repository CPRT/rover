import time

odrvs = [odrv0, odrv1, odrv2, odrv3, odrv4, odrv5, odrv6, odrv7]

# Track each ODrive by serial number
status = {}

# Start calibration on all ODrives first
for odrv in odrvs:
    serial = odrv.serial_number
    print(f"[{serial}] Starting calibration")

    odrv.axis0.config.enable_watchdog = False
    odrv.clear_errors()

    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    status[serial] = {
        "odrv": odrv,
        "done": False,
        "last_print": None,
    }

# Poll all ODrives until all are done
while not all(info["done"] for info in status.values()):
    for serial, info in status.items():
        if info["done"]:
            continue

        odrv = info["odrv"]
        axis = odrv.axis0

        current_state = axis.current_state
        procedure_result = axis.procedure_result
        active_errors = axis.active_errors
        disarm_reason = axis.disarm_reason

        state_tuple = (
            current_state,
            procedure_result,
            active_errors,
            disarm_reason,
        )

        # Only print when something changes
        if state_tuple != info["last_print"]:
            print(
                f"[{serial}]",
                "state:",
                current_state,
                "procedure_result:",
                procedure_result,
                "active_errors:",
                active_errors,
                "disarm_reason:",
                disarm_reason,
            )
            info["last_print"] = state_tuple

        # Calibration finished when axis returns to IDLE
        if current_state == AXIS_STATE_IDLE:
            info["done"] = True

            print(
                f"[{serial}] finished:",
                "procedure_result:",
                procedure_result,
                "active_errors:",
                active_errors,
                "disarm_reason:",
                disarm_reason,
            )

    time.sleep(0.5)

# Check results after all have finished
failed = []

for serial, info in status.items():
    odrv = info["odrv"]
    axis = odrv.axis0

    if axis.procedure_result != PROCEDURE_RESULT_SUCCESS:
        failed.append(
            (
                serial,
                axis.procedure_result,
                axis.active_errors,
                axis.disarm_reason,
            )
        )

# Re-enable watchdogs
for serial, info in status.items():
    odrv = info["odrv"]
    odrv.axis0.config.enable_watchdog = True

if failed:
    print("\nCalibration failures:")
    for serial, procedure_result, active_errors, disarm_reason in failed:
        print(
            f"[{serial}]",
            "procedure_result:",
            procedure_result,
            "active_errors:",
            active_errors,
            "disarm_reason:",
            disarm_reason,
        )

    raise Exception("One or more calibrations failed")

# Save configs only if every calibration succeeded
for serial, info in status.items():
    print(f"[{serial}] Saving configuration")
    info["odrv"].save_configuration()

print("All calibrations completed successfully")
