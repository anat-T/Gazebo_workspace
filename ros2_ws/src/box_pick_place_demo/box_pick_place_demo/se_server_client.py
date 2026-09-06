"""Optional flourish: confirm a successful grasp against the existing
SE_server (/estimate), reusing its contract directly (same multipart shape
as S3EServerStateEstimator.__call__ in
clair_robotics_stack/planning/tamp/s3e_server_estimator.py) without pulling
in unified-planning - just a hand-written PDDL domain/problem string, since
we only ever ask about one predicate.

Not gating mission success: SE_server needs no changes and doesn't need to
be running for the rest of the demo to work; this is purely a logged sanity
check that reuses the existing camera+state-estimation infrastructure.
"""
import io

import requests

_DOMAIN = """(define (domain box-pick)
  (:requirements :strips)
  (:predicates (holding ?b))
  (:constants box)
)"""

_PROBLEM = """(define (problem box-pick-check)
  (:domain box-pick)
  (:objects box)
  (:init)
  (:goal (holding box))
)"""


def check_holding_box(pil_image, server_url: str = "http://localhost:8000", timeout: float = 30.0) -> bool:
    """POST one frame to SE_server and report whether it thinks the box is
    being held. Returns False (rather than raising) on any connection error
    so a demo run doesn't fail just because SE_server isn't up."""
    try:
        buf = io.BytesIO()
        pil_image.save(buf, format="PNG")
        buf.seek(0)
        resp = requests.post(
            f"{server_url.rstrip('/')}/estimate",
            files=[("observations", ("frame.png", buf, "image/png"))],
            data={"observation_type": "image", "domain": _DOMAIN, "problem": _PROBLEM},
            timeout=timeout,
        )
        resp.raise_for_status()
        true_predicates = resp.json().get("task_state", [])
        return "holding(box)" in true_predicates
    except requests.RequestException as exc:
        print(f"se_server_client: SE_server unreachable/failed ({exc}); skipping check")
        return False
