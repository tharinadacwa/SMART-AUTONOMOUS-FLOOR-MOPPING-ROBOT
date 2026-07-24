# maps/

The map is **frozen**. That is what makes a cleaning run repeatable: same map ->
same coverage path -> same run, every time.

| File | Made by | What it is |
|---|---|---|
| `home.pgm` / `home.yaml` | `map_saver_cli`, phase 1 | the raw map |
| `home_qa.txt` | `prepare_map.py` | **READ THIS.** Unreachable rooms, bottleneck widths, time estimate |
| `home_coverage_preview.png` | `prepare_map.py` | **LOOK AT THIS.** The exact path the robot will drive |
| `home_coverage_path.yaml` | `prepare_map.py` | the path itself; fed to `coverage_server` |
| `home_keepout_template.pgm` | `prepare_map.py` | paint on this in GIMP to add no-go zones |
| `home_keepout.pgm` / `.yaml` | you, in GIMP | the no-go mask (optional) |

## Making a keepout mask

1. Open `home_keepout_template.pgm` in GIMP.
2. Paint **pure black** (value 0) over anywhere the robot must never go — the top
   of a staircase, the pet's water bowl, a nest of cables, a rug it bogs down in.
3. Export as `home_keepout.pgm` (raw/binary PGM).
4. Write `home_keepout.yaml` — **copy `resolution` and `origin` from `home.yaml`
   EXACTLY**, or the mask lands in the wrong place:

```yaml
image: home_keepout.pgm
mode: trinary
resolution: 0.05           # MUST match home.yaml
origin: [-8.14, -7.81, 0]  # MUST match home.yaml. Third value MUST be 0.0
negate: 0                  # (Costmap2D has no orientation)
occupied_thresh: 0.65
free_thresh: 0.196
```

5. Regenerate the path so the **plan itself** skips those areas:

```bash
ros2 run minibot_coverage prepare_map.py \
    --map maps/home.yaml --start 0 0 --keepout maps/home_keepout.pgm
```

6. Launch with `use_keepout:=true mask:=/abs/path/maps/home_keepout.yaml`

> Pass the mask to **both** `prepare_map.py` **and** the launch file. The first
> keeps keepout zones out of the coverage *plan*; the second stops Nav2 routing
> *through* them on connector moves between lanes.
