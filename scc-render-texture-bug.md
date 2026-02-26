# Bug: UI Overlays Disappear When Longitudinal Control is Engaged

## Problem

When the comma 3X device engages longitudinal (speed) control, ALL UI overlays disappear:
- Driver Monitoring (DM) icon (bottom-left face icon)
- Alert banners
- Colored steering border (green/cyan)
- All other HUD elements rendered after the HUD renderer

The camera view continues to work fine. When only lateral (steering) control is active via MADS, everything renders correctly.

## Root Cause

**File:** `selfdrive/ui/sunnypilot/onroad/smart_cruise_control.py`
**Method:** `SmartCruiseControlRenderer._draw_icon()` (lines 59-105)

The method uses `rl.begin_texture_mode()` / `rl.end_texture_mode()` to render SCC labels ("SCC-V", "SCC-M") into an offscreen 256x128 render texture. It also uses a custom blend mode (`rl_set_blend_factors` + `BLEND_CUSTOM`) to create a "punch-through" transparent text effect.

The call to `rl.end_texture_mode()` internally resets the OpenGL viewport, projection matrix, and modelview matrix. This disrupts the rendering context established by `begin_scissor_mode()` in `augmented_road_view.py`, making ALL subsequent draw calls in the same frame invisible.

### Why It Only Happens When Engaged

The SCC map controller (`sunnypilot/selfdrive/controls/lib/smart_cruise_control/map_controller.py`) has a state machine that requires `long_enabled == True` to enter any enabled state:
- Line 209: `if not self.long_enabled or not self.enabled: self.state = MapState.disabled`
- Line 235: State only enters `enabled` when `long_enabled and self.enabled`

In **lateral-only mode**: `long_enabled = False` -> `_draw_icon()` is never called -> no GL corruption
In **full engagement**: `long_enabled = True` -> `_draw_icon()` IS called -> `end_texture_mode()` corrupts GL state

## Rendering Pipeline (for reference)

```
augmented_road_view.py _render():
  begin_scissor_mode(content_rect)        # Sets up clipping
    CameraView._render()                  # Camera via EGL shader (works fine)
    model_renderer.render()               # Lane lines (works fine)
    update_fade_out_bottom_overlay()       # Fade effect (works fine)
    hud_renderer.render()                 # <-- Contains the problem
      super()._render()                   # Speed display, etc. (works fine)
      torque_bar.render()                 # (works fine)
      speed_limit_renderer.render()       # (works fine)
      smart_cruise_control_renderer.render()  # <-- _draw_icon() CORRUPTS GL STATE HERE
      turn_signal_controller.render()     # INVISIBLE after corruption
      ...
    alert_renderer.render()               # INVISIBLE
    driver_state_renderer.render()        # INVISIBLE
  end_scissor_mode()
  _draw_border(rect)                      # INVISIBLE
```

## How It Was Debugged

1. Initially suspected alertSize visibility condition -> ruled out (bypassed, still hidden)
2. Initially suspected SLA renderer -> ruled out (disabled, still hidden)
3. Discovered issue affects ALL overlays including border (drawn OUTSIDE scissor mode)
4. Added debug rectangles at each stage of the pipeline (A/B/C/D markers)
5. Found that markers BEFORE `hud_renderer.render()` were visible, AFTER were not
6. Added markers between fade_overlay and hud_renderer (E/F markers)
7. Confirmed `hud_renderer.render()` was the culprit
8. Architect analysis identified `SmartCruiseControlRenderer._draw_icon()` as the specific method

## Fix Applied (Fix #1 - Direct Rendering)

Replaced the render texture approach with direct on-screen drawing:

**Before (broken):**
```python
rl.begin_texture_mode(self.scc_tex)          # Switch to offscreen texture
rl.clear_background(rl.Color(0, 0, 0, 0))
rl.draw_rectangle_rounded(...)               # Draw colored box
rl.rl_set_blend_factors(...)                 # Custom blend for punch-through
rl.rl_set_blend_mode(rl.BLEND_CUSTOM)
rl.draw_text_ex(...)                         # Draw text (punches hole)
rl.rl_set_blend_mode(rl.BLEND_ALPHA)
rl.end_texture_mode()                        # <-- CORRUPTS GL STATE
rl.draw_texture_pro(self.scc_tex.texture, ...) # Draw texture to screen
```

**After (fixed):**
```python
# Draw directly on screen - no render texture needed
rl.draw_rectangle_rounded(...)               # Draw colored box at screen position
rl.draw_text_ex(...)                         # Draw dark text on top
```

The visual difference is minimal: text is drawn as dark color on the colored box instead of being a transparent punch-through. The functional behavior is identical.

## Alternative Fix Options

### Fix #2: Pre-render Outside Scissor Mode
Move the `begin_texture_mode`/`end_texture_mode` rendering to BEFORE `begin_scissor_mode()` in `augmented_road_view.py`. Then only call `draw_texture_pro` inside the scissor mode. This preserves the punch-through effect but requires splitting the SCC renderer into pre-render and draw phases.

### Fix #3: State Restoration (band-aid)
After `end_texture_mode()`, explicitly re-establish the scissor mode and rendering state:
```python
rl.end_texture_mode()
# Re-establish scissor that end_texture_mode disrupted
rl.begin_scissor_mode(int(content_rect.x), int(content_rect.y),
                      int(content_rect.width), int(content_rect.height))
```
This is fragile and doesn't fully address the matrix reset.

## Files Involved

| File | Role |
|------|------|
| `selfdrive/ui/sunnypilot/onroad/smart_cruise_control.py` | **THE BUG** - render texture corrupts GL |
| `selfdrive/ui/onroad/augmented_road_view.py` | Main render pipeline with scissor mode |
| `selfdrive/ui/sunnypilot/onroad/hud_renderer.py` | Calls SCC renderer within HUD chain |
| `selfdrive/ui/onroad/driver_state.py` | DM icon (victim of corruption) |
| `selfdrive/ui/onroad/alert_renderer.py` | Alert banners (victim of corruption) |
| `sunnypilot/selfdrive/controls/lib/smart_cruise_control/map_controller.py` | Gates SCC on `long_enabled` |

## Date

2026-02-26
