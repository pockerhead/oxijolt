//! Jolt's debug assertions, in a native build with the `asserts` feature, print their failure and
//! abort the process. A child process makes a raw call that fails an assertion; without asserts
//! the same call is well defined and the child succeeds.

// The thread and process tests are compiled out on wasm32; their helpers and imports stay.
#![cfg_attr(target_arch = "wasm32", allow(dead_code, unused_imports))]

use std::process::Command;

use oxijolt::Shape;
use oxijolt_sys::{JPH_Quat, JPH_Quat_GetAxisAngle, JPH_Vec3, ASSERTS_ENABLED};

/// Set in the child process that makes the failing call.
const CHILD_ENV: &str = "OXIJOLT_ASSERT_PROBE";

/// How many bytes of the child's stdout and stderr a failure message quotes, from the end.
const OUTPUT_TAIL: usize = 4000;

#[test]
#[ignore = "run as a child process by a_failed_jolt_assertion_aborts_with_its_message"]
fn assertion_child() {
    if std::env::var_os(CHILD_ENV).is_none() {
        return;
    }
    // Initializes Jolt, and installs the assertion handler, through the safe API.
    Shape::new_sphere(1.0).unwrap();
    let not_normalized = JPH_Quat {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 2.0,
    };
    let mut axis = JPH_Vec3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };
    let mut angle = 0.0;
    // SAFETY: all three pointers are live locals; the function reads one quaternion and writes the
    // two outputs. Without asserts Jolt takes its `abs_w >= 1` branch (`Quat.inl:172-176`); with
    // asserts `JPH_ASSERT(IsNormalized())` fails first (`Quat.inl:171`).
    unsafe { JPH_Quat_GetAxisAngle(&not_normalized, &mut axis, &mut angle) };
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn a_failed_jolt_assertion_aborts_with_its_message() {
    let output = Command::new(std::env::current_exe().unwrap())
        .args([
            "assertion_child",
            "--exact",
            "--ignored",
            "--test-threads=1",
            "--nocapture",
        ])
        .env(CHILD_ENV, "1")
        .output()
        .unwrap();
    let tail = |bytes: &[u8]| {
        String::from_utf8_lossy(&bytes[bytes.len().saturating_sub(OUTPUT_TAIL)..]).into_owned()
    };
    let (stdout, stderr) = (tail(&output.stdout), tail(&output.stderr));
    let report = format!("{}\nstdout:\n{stdout}\nstderr:\n{stderr}", output.status);
    if ASSERTS_ENABLED {
        assert!(!output.status.success(), "the child survived: {report}");
        for expected in ["Jolt assertion failed", "Quat.inl", "IsNormalized()"] {
            assert!(stderr.contains(expected), "no {expected:?}: {report}");
        }
    } else {
        assert!(output.status.success(), "the child failed: {report}");
    }
}
